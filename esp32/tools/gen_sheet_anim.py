#!/usr/bin/env python3
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert an arbitrary sprite sheet into avatar/happy_anim.c.

gen_happy_anim.py handles Meta's own jollybot.gif, which is pixel art drawn on
a 5-pixel grid: it samples one colour per art pixel and lets the firmware
scale each one back up. A smooth illustration, like the desktop pets' sprite
sheets, has no such grid and would come out as noise through that pipeline.

This script instead takes a sheet of separated frames, cuts each frame out on
the sheet's cell grid, downscales it to the panel avatar size, quantises the
colours, and emits the same happy_anim.c/h format the firmware already draws.

Usage:
    gen_sheet_anim.py SPRITESHEET [--cols N] [--rows N] [--frames-per-row ...]
                                  [--rows-to-use ...] [--size WxH] [--colors N]
                                  [--frame-ms N] [--out PATH]

Everything is detected from the sheet when the flags are omitted: a regular
grid is assumed, so cells are sheet_size / (cols, rows).
"""

from __future__ import annotations

import argparse
import pathlib
import sys

from PIL import Image, ImageFilter


def rgb565_be(rgb: tuple[int, int, int]) -> int:
    r, g, b = rgb
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    # Panel expects the most significant byte first.
    return ((v & 0xFF) << 8) | (v >> 8)


def cell_bounds(index: int, cols: int, cell_w: int, cell_h: int):
    row, col = divmod(index, cols)
    return col * cell_w, row * cell_h, (col + 1) * cell_w, (row + 1) * cell_h


def frame_is_blank(cell: Image.Image, alpha_cutoff: int = 8) -> bool:
    """A cell with almost no opaque pixels is an empty grid slot."""
    alpha = cell.getchannel("A")
    # Any pixel above the cutoff counts as content.
    return alpha.point(lambda a: 255 if a > alpha_cutoff else 0).getbbox() is None


def trim_and_scale(cell: Image.Image, size: tuple[int, int],
                   alpha_cutoff: int = 8) -> Image.Image:
    """Crop the frame to its content, then fit it centred on a transparent
    canvas of `size`, keeping the aspect ratio so the character is not
    stretched when the frame is taller or wider than the others.

    The source art is crisp line work; a smooth downscale then a nearest-
    neighbour upscale by the firmware's LCD_ANIM_SCALE leaves it soft. Sharpen
    after the resize so the edges survive both steps."""
    bbox = cell.getchannel("A").point(
        lambda a: 255 if a > alpha_cutoff else 0).getbbox()
    if bbox:
        cell = cell.crop(bbox)

    tw, th = size
    scale = min(tw / cell.width, th / cell.height)
    new = (max(1, round(cell.width * scale)), max(1, round(cell.height * scale)))
    cell = cell.resize(new, Image.LANCZOS)

    # Hard-cut the edge: the panel has no alpha, so a part-transparent pixel
    # would otherwise be flattened against the background and read as a halo.
    a = cell.getchannel("A")
    a = a.point(lambda v: 255 if v >= 160 else 0)
    cell.putalpha(a)

    # Sharpen, then flatten. convert("RGB") on an RGBA image drops the alpha
    # and substitutes black for anything not opaque, so the sharpened edge ends
    # up with black RGB; the frame is flattened against its own average colour
    # instead, which keeps the silhouette edge the tone of the body.
    opaque = [p[:3] for p in cell.get_flattened_data() if p[3] > 0]
    if opaque:
        n = len(opaque)
        body = (sum(p[0] for p in opaque) // n,
                sum(p[1] for p in opaque) // n,
                sum(p[2] for p in opaque) // n)
    else:
        body = (0, 0, 0)
    flat = Image.new("RGB", new, body)
    flat.paste(cell, (0, 0), cell)
    flat = flat.filter(ImageFilter.UnsharpMask(radius=1.0, percent=90, threshold=3))

    shaped = Image.new("RGBA", size, (0, 0, 0, 0))
    inner = flat.convert("RGBA")
    inner.putalpha(a)
    shaped.paste(inner, ((tw - new[0]) // 2, (th - new[1]) // 2), inner)
    return shaped


def quantise(frames: list[Image.Image], colors: int, alpha_cutoff: int):
    """Build one shared palette across every frame.

    Per-frame palettes would make the colours shimmer as the animation plays,
    so all frames are quantised together."""
    # Composite onto black: the panel has no alpha channel. The firmware maps
    # index 0 to the background, so the only pixels that may take index 0 are
    # those that are actually transparent; everything else must get a real
    # colour even if it is very dark.
    flat = []
    for f in frames:
        bg = Image.new("RGB", f.size, (0, 0, 0))
        bg.paste(f, mask=f.getchannel("A"))
        flat.append(bg)

    # Quantise. The rim highlight is suppressed after the resize, where the
    # silhouette is at its final resolution, but before the palette is built
    # so the pale rim cannot claim palette entries of its own.
    strip = Image.new("RGB", (flat[0].width, flat[0].height * len(flat)))
    for i, f in enumerate(flat):
        strip.paste(f, (0, i * f.height))

    # Count exact colours (the art is already limited), then take the most
    # frequent ones and merge the rest into their nearest neighbour.
    counts: dict[tuple[int, int, int], int] = {}
    for f in flat:
        for px in f.getdata():
            counts[px] = counts.get(px, 0) + 1

    # Keep the darkest colour out of the running: index 0 is the background.
    ranked = sorted(counts.items(), key=lambda kv: -kv[1])
    chosen: list[tuple[int, int, int]] = [(0, 0, 0)]
    for rgb, _ in ranked:
        if len(chosen) >= colors:
            break
        if rgb == (0, 0, 0):
            continue
        # Skip colours that are nearly the same as one already chosen.
        if any(abs(rgb[0] - c[0]) + abs(rgb[1] - c[1]) + abs(rgb[2] - c[2]) < 12
               for c in chosen):
            continue
        chosen.append(rgb)
    pal_rgb = chosen

    def nearest(rgb: tuple[int, int, int]) -> int:
        best, best_d = 0, 1 << 30
        for i, c in enumerate(pal_rgb):
            d = (c[0] - rgb[0]) ** 2 + (c[1] - rgb[1]) ** 2 + (c[2] - rgb[2]) ** 2
            if d < best_d:
                best, best_d = i, d
        return best

    cache: dict[tuple[int, int, int], int] = {}
    out = []
    for f, rgb_frame in zip(frames, flat):
        alpha = f.getchannel("A")
        idx = []
        for y in range(f.height):
            for x in range(f.width):
                if alpha.getpixel((x, y)) <= alpha_cutoff:
                    idx.append(0)          # transparent -> background
                    continue
                rgb = rgb_frame.getpixel((x, y))
                if rgb not in cache:
                    cache[rgb] = nearest(rgb)
                idx.append(cache[rgb])
        out.append(idx)
    return pal_rgb, out


def suppress_edge_highlight(img: Image.Image, alpha_cutoff: int = 160,
                            keep: int = 3) -> Image.Image:
    """Pull the light rim on the silhouette back toward the body colour.

    The character is drawn with a pale highlight along its outline: pixels that
    are much lighter than everything around them, sitting right against the
    transparent background. It reads as a white halo on a panel that has no
    alpha to blend it into. Walk the silhouette and recolour any rim pixel to
    the nearest body colour, leaving `keep` pixels of the drawing's own dark
    outline untouched."""
    px = img.load()
    w, h = img.size
    opaque = [[px[x, y][3] >= alpha_cutoff for x in range(w)] for y in range(h)]

    def is_rim(x: int, y: int) -> bool:
        if not opaque[y][x]:
            return False
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h) or not opaque[ny][nx]:
                return True
        return False

    rims = [(x, y) for y in range(h) for x in range(w) if is_rim(x, y)]
    if not rims:
        return img

    rim_set = set(rims)
    for x, y in rims:
        # Look inward for a body colour. The rim on a curved edge can be two or
        # three pixels thick with the body below it, so sweep a window rather
        # than a single step. Prefer the darkest plausible body pixel: the
        # highlight is lighter than the drawing's own outline, and the outline
        # is what the rim should sit against.
        target = None
        best_lum = 1 << 30
        for dist in range(1, keep + 8):
            for dy in range(-dist, dist + 1):
                for dx in range(-dist, dist + 1):
                    if max(abs(dx), abs(dy)) != dist:
                        continue
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < w and 0 <= ny < h):
                        continue
                    if not opaque[ny][nx] or (nx, ny) in rim_set:
                        continue
                    r2, g2, b2, _ = px[nx, ny]
                    lum = (r2 + g2 + b2) // 3
                    if lum < best_lum:
                        best_lum = lum
                        target = (r2, g2, b2)
            if target is not None:
                break
        if target is None:
            continue
        r, g, b, a = px[x, y]
        # Rewrite anything lighter than the body it borders. The threshold is
        # low because the highlight is a subtle tint, not a bright white.
        lum_r = (r + g + b) // 3
        if lum_r > best_lum + 6:
            px[x, y] = (target[0], target[1], target[2], a)
        else:
            px[x, y] = ((r + target[0] * 2) // 3,
                        (g + target[1] * 2) // 3,
                        (b + target[2] * 2) // 3, a)
    return img


def scavenge_white_fringe(img: Image.Image, alpha_cutoff: int = 24) -> Image.Image:
    """Drop the sheet's invisible white edge pixels.

    Every silhouette in this sheet carries pixels that are pure white at
    near-zero alpha. They are invisible in the source but become a bright
    white outline once the alpha is flattened for the panel, so remove them
    before anything else touches the image."""
    px = img.load()
    for y in range(img.height):
        for x in range(img.width):
            r, g, b, a = px[x, y]
            if a <= alpha_cutoff:
                px[x, y] = (0, 0, 0, 0)
    return img


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("sheet")
    ap.add_argument("--cols", type=int, default=8)
    ap.add_argument("--rows", type=int, default=11)
    ap.add_argument("--size", default="56x59")
    ap.add_argument("--colors", type=int, default=64)
    ap.add_argument("--frame-ms", type=int, default=120)
    ap.add_argument("--out", default=None)
    ap.add_argument("--alpha-cutoff", type=int, default=8)
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    sheet = Image.open(args.sheet).convert("RGBA")
    sheet = scavenge_white_fringe(sheet)
    cell_w = sheet.width // args.cols
    cell_h = sheet.height // args.rows

    frames = []
    for i in range(args.cols * args.rows):
        box = cell_bounds(i, args.cols, cell_w, cell_h)
        cell = sheet.crop(box)
        if frame_is_blank(cell, args.alpha_cutoff):
            continue
        cell = trim_and_scale(cell, (w, h), args.alpha_cutoff)
        cell = suppress_edge_highlight(cell)
        frames.append(cell)

    if not frames:
        sys.exit("no non-empty frames found; check --cols/--rows")

    palette, indices = quantise(frames, args.colors, args.alpha_cutoff)

    out = pathlib.Path(args.out) if args.out else (
        pathlib.Path(__file__).resolve().parent.parent / "avatar" / "happy_anim.c")

    lines = [
        "// Copyright (c) Meta Platforms, Inc. and affiliates.",
        "",
        f"// Generated by tools/gen_sheet_anim.py from {pathlib.Path(args.sheet).name}."
        " Do not edit.",
        '#include "happy_anim.h"',
        "",
        "#if CONFIG_HOMEHUB_DISPLAY",
        "",
        f"const uint16_t happy_anim_palette[{len(palette)}] = {{",
    ]
    for c in palette:
        lines.append(f"    0x{rgb565_be(c):04x},  // #{c[0]:02x}{c[1]:02x}{c[2]:02x}")
    lines += [
        "};",
        "",
        "const uint8_t happy_anim_frames[HAPPY_ANIM_FRAMES]"
        "[HAPPY_ANIM_HEIGHT * HAPPY_ANIM_WIDTH] = {",
    ]
    for frame in indices:
        lines.append("    {")
        for y in range(h):
            row = frame[y * w:(y + 1) * w]
            lines.append("        " + ",".join(str(v) for v in row) + ",")
        lines.append("    },")
    lines += ["};", "", "#endif", ""]
    out.write_text("\n".join(lines))

    header = [
        "// Copyright (c) Meta Platforms, Inc. and affiliates.",
        "",
        f"// Generated by tools/gen_sheet_anim.py from {pathlib.Path(args.sheet).name}."
        " Do not edit.",
        "#pragma once",
        "",
        "#include <stdint.h>",
        '#include "sdkconfig.h"',
        "",
        f"#define HAPPY_ANIM_WIDTH     {w}",
        f"#define HAPPY_ANIM_HEIGHT    {h}",
        f"#define HAPPY_ANIM_FRAMES    {len(indices)}",
        f"#define HAPPY_ANIM_FRAME_MS  {args.frame_ms}",
        "",
        "#if CONFIG_HOMEHUB_DISPLAY",
        f"extern const uint16_t happy_anim_palette[{len(palette)}];",
        "extern const uint8_t happy_anim_frames[HAPPY_ANIM_FRAMES]"
        "[HAPPY_ANIM_HEIGHT * HAPPY_ANIM_WIDTH];",
        "#endif",
        "",
    ]
    out.with_suffix(".h").write_text("\n".join(header))
    print(f"{out}: {len(indices)} frames, {len(palette)} colours, {w}x{h}")


if __name__ == "__main__":
    main()
