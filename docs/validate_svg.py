#!/usr/bin/env python3
"""Validate SVG diagrams in index.html"""

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# HTML entities that are NOT valid in XML — replace with Unicode
_HTML_ONLY_ENTITIES = {
    'le': '\u2264', 'ge': '\u2265',
    'rarr': '\u2192', 'larr': '\u2190', 'darr': '\u2193', 'uarr': '\u2191',
    'swarr': '\u2199', 'searr': '\u2198', 'nwarr': '\u2196', 'nearr': '\u2197',
    'rArr': '\u21d2', 'lArr': '\u21d0',
    'alpha': '\u03b1', 'beta': '\u03b2', 'gamma': '\u03b3', 'delta': '\u03b4',
    'epsilon': '\u03b5',
    'middot': '\u00b7',
    'times': '\u00d7',
    'uarr': '\u2191',
    'ndash': '\u2013', 'mdash': '\u2014',
    'nbsp': '\u00a0',
    'bull': '\u2022',
    'deg': '\u00b0',
    'plusmn': '\u00b1',
    'micro': '\u00b5',
    'hellip': '\u2026',
    'copy': '\u00a9',
    'trade': '\u2122',
    'reg': '\u00ae',
}

# Standard XML entities — leave intact
_XML_ENTITIES = frozenset(['amp', 'lt', 'gt', 'quot', 'apos'])

_ENTITY_RE = re.compile(r'&(\w+);')

# ASCII replacements for Unicode chars that can't print on Windows cp1252
_UNICODE_ASCII = {
    '\u2192': '->', '\u2190': '<-', '\u2191': '^', '\u2193': 'v',
    '\u2199': '<-v', '\u2198': 'v->',
    '\u2264': '<=', '\u2265': '>=',
    '\u00b7': '*',
    '\u2013': '-', '\u2014': '--',
    '\u03b1': 'alpha',
    '\u03b5': 'epsilon',
    '\u00d7': 'x',
    '\u00b0': 'deg',
    '\u00a0': ' ',
    '\u2022': '*',
    '\u2026': '...',
}


def _sanitize(text):
    """Replace Unicode characters with ASCII equivalents for console output."""
    for ch, repl in _UNICODE_ASCII.items():
        text = text.replace(ch, repl)
    return text


def _replace_html_only_entities(text):
    """Replace HTML-only entities with Unicode, keeping XML entities intact."""
    def repl(m):
        name = m.group(1)
        if name in _XML_ENTITIES:
            return m.group(0)  # keep as-is; XML parser handles it
        if name in _HTML_ONLY_ENTITIES:
            return _HTML_ONLY_ENTITIES[name]
        return m.group(0)  # unknown — leave as-is (will cause XML error, but that's OK)
    return _ENTITY_RE.sub(repl, text)


def extract_svg_blocks(html_text):
    """Extract SVG blocks from HTML using regex."""
    pattern = r'<svg\b[^>]*>.*?</svg>'
    return [(m.group(0), m.start()) for m in re.finditer(pattern, html_text, re.DOTALL)]


def find_section_name(html_text, pos):
    """Find section name by scanning backwards for the most specific context."""
    before = html_text[:pos]

    # 1) Check if inside a module-card (highest priority)
    card_starts = list(re.finditer(r'<div[^>]*class="module-card[^"]*"', before))
    if card_starts:
        last_card = card_starts[-1]
        card_open = last_card.start()
        # Find h3 inside this card (between card open and SVG position)
        inner = before[card_open:pos]
        h3_m = re.search(r'<h3[^>]*>(.*?)</h3>', inner, re.DOTALL)
        if h3_m:
            title = re.sub(r'<[^>]+>', '', h3_m.group(1)).strip()
            if title:
                return title

    # 2) Check if inside a tab-content
    tab_matches = list(re.finditer(
        r'<div[^>]*class="tab-content[^"]*"[^>]*id="([^"]*)"', before
    ))
    if tab_matches:
        # Search backwards to find which tab-content the SVG is inside
        # by tracking opening/closing div nesting from the nearest tab-content
        for tm in reversed(tab_matches):
            tab_id = tm.group(1)
            # Look for the tab button that matches this id
            btn_m = re.search(
                rf'data-tab="{re.escape(tab_id)}"[^>]*>(.*?)</button>', html_text
            )
            if btn_m:
                title = re.sub(r'<[^>]+>', '', btn_m.group(1)).strip()
                if title:
                    return title

    # 3) h2.section-title
    h2s = list(re.finditer(
        r'<h2[^>]*class="section-title"[^>]*>(.*?)</h2>', before, re.DOTALL
    ))
    if h2s:
        title = re.sub(r'<[^>]+>', '', h2s[-1].group(1)).strip()
        if title:
            return title

    # 4) any h3
    h3s = list(re.finditer(r'<h3[^>]*>(.*?)</h3>', before, re.DOTALL))
    if h3s:
        title = re.sub(r'<[^>]+>', '', h3s[-1].group(1)).strip()
        if title:
            return title

    # 5) any h4
    h4s = list(re.finditer(r'<h4[^>]*>(.*?)</h4>', before, re.DOTALL))
    if h4s:
        title = re.sub(r'<[^>]+>', '', h4s[-1].group(1)).strip()
        if title:
            return title

    return "Unknown"


def parse_num(val, default=0.0):
    if val is None:
        return default
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def parse_path_d(d):
    """Extract start and end coordinates from a path d attribute."""
    if not d:
        return None, None
    cmds = re.findall(r'([ML])\s*([-\d.e]+)\s*[, \t]*\s*([-\d.e]+)', d)
    if not cmds:
        return None, None
    first = cmds[0]
    last = cmds[-1]
    return (float(first[1]), float(first[2])), (float(last[1]), float(last[2]))


def parse_viewbox(svg_el):
    vb = svg_el.get('viewBox', '')
    if not vb:
        return None, None, None, None
    parts = vb.strip().split()
    if len(parts) == 4:
        return float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
    return None, None, None, None


def el_tag(el):
    tag = el.tag
    if '}' in tag:
        return tag.split('}', 1)[1]
    return tag


def collect_elements(svg_el):
    rects, texts, paths, lines = [], [], [], []
    for el in svg_el.iter():
        tag = el_tag(el)
        if tag == 'rect':
            rx, ry = parse_num(el.get('x')), parse_num(el.get('y'))
            rw, rh = parse_num(el.get('width')), parse_num(el.get('height'))
            rects.append((rx, ry, rw, rh, el))
        elif tag == 'text':
            tx, ty = parse_num(el.get('x')), parse_num(el.get('y'))
            content = _sanitize(''.join(el.itertext()).strip())
            texts.append((tx, ty, content, el))
        elif tag == 'path':
            d = el.get('d', '')
            marker = el.get('marker-end', '')
            paths.append((d, marker, el))
        elif tag == 'line':
            x1 = parse_num(el.get('x1'))
            y1 = parse_num(el.get('y1'))
            x2 = parse_num(el.get('x2'))
            y2 = parse_num(el.get('y2'))
            marker = el.get('marker-end', '')
            lines.append((x1, y1, x2, y2, marker, el))
    return rects, texts, paths, lines


def get_rect_label(rect, texts):
    rx, ry, rw, rh = rect[:4]
    cx, cy = rx + rw / 2, ry + rh / 2
    best, best_dist = '', float('inf')
    for tx, ty, text, _ in texts:
        dist = abs(tx - cx) + abs(ty - cy) * 2
        if dist < best_dist and abs(tx - cx) < rw and abs(ty - cy) < rh:
            best_dist = dist
            best = text
    return best if best else f"rect@{rx:.0f}"


def point_in_rect(px, py, rx, ry, rw, rh):
    return rx <= px <= rx + rw and ry <= py <= ry + rh


def main():
    html_path = Path(__file__).parent / 'index.html'
    if not html_path.exists():
        print(f"ERROR: {html_path} not found")
        sys.exit(1)

    html_text = html_path.read_text(encoding='utf-8')

    blocks = extract_svg_blocks(html_text)
    if not blocks:
        print("No SVG blocks found")
        sys.exit(1)

    total_diagrams = len(blocks)
    passed = failed = 0
    total_ac = total_bb = total_co = total_acv = 0

    for svg_raw, svg_pos in blocks:
        section = find_section_name(html_text, svg_pos)

        # Replace HTML-only entities with Unicode (keep XML entities intact)
        svg_xml = _replace_html_only_entities(svg_raw)

        try:
            svg_el = ET.fromstring(svg_xml)
        except ET.ParseError as e:
            print(f"=== DIAGRAM: {_sanitize(section)} ===")
            print(f"  PARSE ERROR: {e}")
            failed += 1
            print()
            continue

        vb_x, vb_y, vb_w, vb_h = parse_viewbox(svg_el)
        rects, texts, paths, lines = collect_elements(svg_el)

        # Main rects: exclude tiny decorative rects and outline-only callout boxes
        main_rects = [
            r for r in rects
            if r[2] >= 40 and r[3] >= 16 and r[4].get('fill', 'none') != 'none'
        ]

        ac_issues, bb_issues, co_issues, acv_issues = [], [], [], []

        # --- Collect all arrows (paths/lines with marker-end) ---
        arrows = []
        for d, marker, el in paths:
            if marker:
                s, e = parse_path_d(d)
                if s and e:
                    arrows.append((s[0], s[1], e[0], e[1], d, marker))
        for x1, y1, x2, y2, marker, el in lines:
            if marker:
                arrows.append((x1, y1, x2, y2, f"L{x1} {y1} {x2} {y2}", marker))

        # --- AC: Arrow Connection Validation ---
        for sx, sy, ex, ey, d, marker in arrows:
            if abs(sy - ey) > 1:
                continue  # only horizontal arrows

            is_ltr = ex > sx  # left-to-right or right-to-left

            matched = False
            for rx, ry, rw, rh, el in rects:
                if abs(ry + rh / 2 - sy) > rh / 2 + 15:
                    continue

                if is_ltr:
                    if abs(ex - rx) < 5 or abs(sx - (rx + rw)) < 5:
                        matched = True
                        break
                else:
                    if abs(ex - (rx + rw)) < 5 or abs(sx - rx) < 5:
                        matched = True
                        break

            if matched:
                continue

            # Report misalignment
            if is_ltr:
                best_right = None
                best_dist = float('inf')
                for rx, ry, rw, rh, el in rects:
                    if abs(ry + rh / 2 - sy) <= rh / 2 + 15 and rx > ex:
                        gap = rx - ex
                        if gap < best_dist:
                            best_dist = gap
                            best_right = (rx, ry, rw, rh)
                if best_right is not None and 2 < best_dist < 200:
                    ac_issues.append(
                        f"arrow ends at x={ex:.0f} but next rect starts at x={best_right[0]:.0f} "
                        f"(gap={best_dist:.0f}px)"
                    )

                best_left = None
                best_ldist = float('inf')
                for rx, ry, rw, rh, el in rects:
                    if abs(ry + rh / 2 - sy) <= rh / 2 + 15:
                        right_edge = rx + rw
                        if right_edge <= sx:
                            gap = sx - right_edge
                            if gap < best_ldist:
                                best_ldist = gap
                                best_left = (rx, ry, rw, rh)
                if best_left is not None and 2 < best_ldist < 200:
                    ac_issues.append(
                        f"arrow starts at x={sx:.0f} but prev rect ends at x="
                        f"{best_left[0] + best_left[2]:.0f} (gap={best_ldist:.0f}px)"
                    )
            else:
                # Right-to-left: start at rect LEFT edge, end at rect RIGHT edge
                best_re = None
                best_rd = float('inf')
                for rx, ry, rw, rh, el in rects:
                    re = rx + rw
                    if abs(ry + rh / 2 - sy) <= rh / 2 + 15 and re < sx:
                        gap = sx - re
                        if 0 < gap < best_rd:
                            best_rd = gap
                            best_re = re
                if best_re is not None and best_rd > 2:
                    ac_issues.append(
                        f"arrow starts at x={sx:.0f} (from right) but nearest "
                        f"rect right edge is at x={best_re:.0f} (gap={best_rd:.0f}px)"
                    )

                best_le = None
                best_ld = float('inf')
                for rx, ry, rw, rh, el in rects:
                    if abs(ry + rh / 2 - sy) <= rh / 2 + 15 and rx > ex:
                        gap = rx - ex
                        if 0 < gap < best_ld:
                            best_ld = gap
                            best_le = rx
                if best_le is not None and best_ld > 2:
                    ac_issues.append(
                        f"arrow ends at x={ex:.0f} (entering from right) but nearest "
                        f"rect left edge is at x={best_le:.0f} (gap={best_ld:.0f}px)"
                    )

        # --- BB: Box Boundary Validation ---
        for rx, ry, rw, rh, el in rects:
            if rw < 30 or rh < 30:
                continue
            for tx, ty, text, tel in texts:
                if not text:
                    continue
                near_cx = abs(tx - (rx + rw / 2)) < rw * 0.6
                near_cy = abs(ty - (ry + rh / 2)) < rh * 0.6
                if not (near_cx and near_cy):
                    continue
                inside_x = rx <= tx <= rx + rw
                inside_y = ry - 5 <= ty <= ry + rh
                if not (inside_x and inside_y):
                    bb_issues.append(
                        f"text '{text[:30]}' at ({tx:.0f},{ty:.0f}) outside rect "
                        f"bounds ({rx:.0f},{ry:.0f},{rw:.0f},{rh:.0f})"
                    )

        # --- CO: Content Overflow ---
        if vb_w is not None:
            for rx, ry, rw, rh, el in rects:
                if rx + rw > vb_x + vb_w + 1 or ry + rh > vb_y + vb_h + 1:
                    co_issues.append(
                        f"rect at ({rx:.0f},{ry:.0f},w={rw:.0f},h={rh:.0f}) "
                        f"extends beyond viewBox ({vb_x:.0f},{vb_y:.0f},{vb_w:.0f},{vb_h:.0f})"
                    )

        # --- ACV: Arrowhead Coverage ---
        rows = {}
        for r in main_rects:
            rx, ry, rw, rh = r[:4]
            key = (round(ry), round(rh))
            rows.setdefault(key, []).append(r)

        for (y_key, h_key), row_rects in rows.items():
            row_rects.sort(key=lambda r: r[0])
            for j in range(len(row_rects) - 1):
                r1, r2 = row_rects[j], row_rects[j + 1]
                r1_right = r1[0] + r1[2]
                r2_left = r2[0]
                gap = r2_left - r1_right
                if gap > 250:
                    continue

                # Skip sibling branches: both rects have arrows going downward
                def rect_has_down_arrow(r):
                    rx, ry, rw, rh = r[:4]
                    for sx, sy, ex, ey, d, marker in arrows:
                        if abs(sx - (rx + rw / 2)) < rw and abs(sy - (ry + rh / 2)) < rh:
                            if ey > sy + 5:
                                return True
                        if abs(sx - (rx + rw)) < 10 and abs(sy - (ry + rh / 2)) < rh / 2:
                            if ey > sy + 5:
                                return True
                    return False

                if rect_has_down_arrow(r1) and rect_has_down_arrow(r2):
                    continue

                connected = False
                for sx, sy, ex, ey, d, marker in arrows:
                    # Check if arrow connects r1 -> r2
                    # Start should be near r1's body (right edge or interior)
                    # End should be near r2's body (left edge or interior)
                    in_r1 = point_in_rect(sx, sy, r1[0], r1[1], r1[2], r1[3])
                    near_r1_right = abs(sx - r1_right) < 30 and abs(sy - (r1[1] + r1[3] / 2)) < r1[3]
                    in_r2 = point_in_rect(ex, ey, r2[0], r2[1], r2[2], r2[3])
                    near_r2_left = abs(ex - r2_left) < 30 and abs(ey - (r2[1] + r2[3] / 2)) < r2[3]

                    start_ok = in_r1 or near_r1_right
                    end_ok = in_r2 or near_r2_left

                    if start_ok and end_ok:
                        connected = True
                        break

                    # Also check reversed direction (r2 -> r1)
                    in_r2_rev = point_in_rect(sx, sy, r2[0], r2[1], r2[2], r2[3])
                    near_r2_left_rev = abs(sx - r2_left) < 30 and abs(sy - (r2[1] + r2[3] / 2)) < r2[3]
                    in_r1_rev = point_in_rect(ex, ey, r1[0], r1[1], r1[2], r1[3])
                    near_r1_right_rev = abs(ex - r1_right) < 30 and abs(ey - (r1[1] + r1[3] / 2)) < r1[3]

                    start_ok_rev = in_r2_rev or near_r2_left_rev
                    end_ok_rev = in_r1_rev or near_r1_right_rev

                    if start_ok_rev and end_ok_rev:
                        connected = True
                        break

                if not connected:
                    name1 = get_rect_label(r1, texts)
                    name2 = get_rect_label(r2, texts)
                    acv_issues.append(
                        f"'{name1}' -> '{name2}': no path connects "
                        f"x={r1_right:.0f} to x={r2_left:.0f}"
                    )

        # --- Output ---
        n_arrows = len(arrows)
        has_issues = bool(ac_issues or bb_issues or co_issues or acv_issues)

        safe_section = _sanitize(section)
        print("=" * 50)
        print(f"  DIAGRAM: {safe_section}")
        print(f"  Rects found: {len(rects)} (main: {len(main_rects)})")
        print(f"  Paths found: {len(paths)}")
        print(f"  Lines found: {len(lines)}")
        print(f"  Texts found: {len(texts)}")

        ac_s = f"OK ({n_arrows} arrows checked)" if not ac_issues else f"FAIL ({len(ac_issues)} issue(s))"
        bb_s = "OK" if not bb_issues else f"FAIL ({len(bb_issues)} issue(s))"
        co_s = "OK" if not co_issues else f"FAIL ({len(co_issues)} issue(s))"
        acv_s = "OK" if not acv_issues else f"FAIL ({len(acv_issues)} issue(s))"

        print(f"  AC: {ac_s}")
        for iss in ac_issues:
            print(f"    AC FAIL: {safe_section}: {iss}")
        print(f"  BB: {bb_s}")
        for iss in bb_issues:
            print(f"    BB FAIL: {iss}")
        print(f"  CO: {co_s}")
        for iss in co_issues:
            print(f"    CO FAIL: {iss}")
        print(f"  ACV: {acv_s}")
        for iss in acv_issues:
            print(f"    ACV FAIL: {iss}")

        # Debug dump
        print(f"  Debug Dump:")
        print(f"    Rects:")
        for rx, ry, rw, rh, el in rects:
            stroke = el.get('stroke', 'none')
            fill = el.get('fill', 'none')
            print(f"      x={rx:.0f} y={ry:.0f} w={rw:.0f} h={rh:.0f}  stroke={stroke}  fill={fill}")
        horiz = [(sx, sy, ex, ey, d, m) for sx, sy, ex, ey, d, m in arrows if abs(sy - ey) < 1]
        if horiz:
            print(f"    Horizontal Arrows ({len(horiz)}):")
            for sx, sy, ex, ey, d, m in horiz:
                print(f"      ({sx:.0f},{sy:.0f}) -> ({ex:.0f},{ey:.0f})  {m}")
        print(f"    Text elements:")
        for tx, ty, text, tel in texts:
            txt = text[:50].replace('\n', '\\n')
            print(f"      ({tx:.0f},{ty:.0f}): '{txt}'")

        if has_issues:
            failed += 1
        else:
            passed += 1

        total_ac += len(ac_issues)
        total_bb += len(bb_issues)
        total_co += len(co_issues)
        total_acv += len(acv_issues)

        print()

    # --- Summary ---
    print("=" * 50)
    print(f"  OVERALL SUMMARY")
    print(f"  Total diagrams: {total_diagrams}")
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")
    print(f"  ---")
    print(f"  Total AC issues: {total_ac}")
    print(f"  Total BB issues: {total_bb}")
    print(f"  Total CO issues: {total_co}")
    print(f"  Total ACV issues: {total_acv}")
    print()

    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
