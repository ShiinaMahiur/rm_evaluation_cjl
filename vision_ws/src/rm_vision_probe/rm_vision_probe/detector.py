"""S1: 传统灯条法装甲板检测（纯函数，节点和离线评估共用）

流程：颜色阈值 → 灯条筛选 → 两两配对（几何约束 + 数字区校验） → 装甲板四角点

为什么这么设计（答辩用）：
  1. 颜色阈值（HSV）在受控光照下快且可解释；缺点是强光/过曝会崩 —— 这正好是
     README「已知失败场景」要写的边界。
  2. 灯条用「高宽比 + 填充率」筛选：装甲板灯条是竖直细长亮条，场地图案是细长线，
     靠填充率和高度下限可以分开。
  3. 配对必须加**数字区校验**：两个灯条之间的区域应该有低饱和中等亮度的数字像素，
     否则会把前哨站等物体的两条装饰灯误配成装甲板（实测这是最大的误检来源）。
"""
import cv2
import numpy as np


class DetectorParams:
    def __init__(self, **kw):
        # 颜色阈值
        self.v_thresh = kw.get("v_thresh", 110)
        self.s_thresh = kw.get("s_thresh", 90)
        self.blue_hue = kw.get("blue_hue", (90, 140))
        self.red_hue = kw.get("red_hue", (12, 168))
        # 灯条筛选
        self.min_area = kw.get("min_area", 12)
        self.min_height = kw.get("min_height", 8)
        self.min_aspect = kw.get("min_aspect", 1.5)      # 高/宽
        self.min_fill = kw.get("min_fill", 0.25)         # 面积 / 外接矩形
        # 配对
        self.max_dh = kw.get("max_dh", 0.35)             # 两灯高度差 / 平均高
        self.max_dy = kw.get("max_dy", 0.55)             # 两灯中心 y 差 / 平均高
        self.gap_ratio = kw.get("gap_ratio", (0.6, 3.0)) # 灯间距 / 平均高
        self.min_digit = kw.get("min_digit", 0.04)       # 数字区像素比例下限
        self.max_armors = kw.get("max_armors", 8)


def _masks(img, p):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    base = (v > p.v_thresh) & (s > p.s_thresh)
    out = {
        "blue": (base & (h > p.blue_hue[0]) & (h < p.blue_hue[1])).astype(np.uint8) * 255,
        "red": (base & ((h < p.red_hue[0]) | (h > p.red_hue[1]))).astype(np.uint8) * 255,
    }
    for k in out:
        out[k] = cv2.morphologyEx(out[k], cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return out


def _lights(mask, p):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        a = cv2.contourArea(c)
        if a < p.min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h < p.min_height or w < 2 or h / w < p.min_aspect:
            continue
        if a / (w * h) < p.min_fill:
            continue
        out.append((x, y, w, h))
    return out


def _digit_score(img, x1, x2, y1, y2):
    """两灯之间区域里"数字像素"（低饱和、中等亮度）的比例"""
    if x2 - x1 < 6 or y2 - y1 < 6:
        return 0.0
    roi = img[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    digit = ((s < 90) & (v > 55) & (v < 200)).astype(np.uint8)
    return float(digit.mean())


PLATE_ASPECT = 0.125 / 0.15   # 装甲板高/宽（与 solver.py 的 PLATE_W/PLATE_H 一致）


def _cross_edges(prof, core_lo, core_hi):
    """1D 亮度剖面 → 50% 电平处的左右亚像素边界（线性插值）"""
    bg_parts = [prof[:max(core_lo, 0)], prof[core_hi + 1:]]
    bg_arr = np.concatenate(bg_parts) if any(x.size for x in bg_parts) else np.array([])
    bg = float(np.median(bg_arr)) if bg_arr.size >= 2 else float(np.percentile(prof, 5))
    peak = float(np.percentile(prof, 95))
    if peak - bg < 8.0:
        return None
    thr = bg + 0.5 * (peak - bg)
    left = right = None
    for i in range(max(core_lo, 1), core_hi + 1):
        if prof[i] >= thr and prof[i - 1] < thr:
            left = i - 1 + (thr - prof[i - 1]) / max(prof[i] - prof[i - 1], 1e-6)
            break
    for i in range(core_hi, core_lo, -1):
        if prof[i] >= thr and prof[i + 1] < thr:
            right = i + (prof[i] - thr) / max(prof[i] - prof[i + 1], 1e-6)
            break
    if left is None or right is None:
        return None
    return left, right


def _lamp_subpix(v, mask, rect):
    """单根灯条亚像素定位 → (cx, cy, w_sub, h_sub)，失败返回 None

    做法：取灯条内部行（列）的 V 通道平均亮度剖面，在「背景亮度 + 50%×峰值差」
    处线性插值找边缘。相比整像素外接矩形，边缘误差从 ±0.5 px 降到 ~±0.1 px，
    对边缘模糊/半饱和像素更稳。
    """
    x, y, w, h = rect
    m = 4
    x0, y0 = max(x - m, 0), max(y - m, 0)
    x1, y1 = min(x + w + m, mask.shape[1]), min(y + h + m, mask.shape[0])
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    msub = mask[y0:y1, x0:x1] > 0
    if msub.sum() < 8:
        return None
    vsub = np.asarray(v[y0:y1, x0:x1], dtype=np.float32)

    row_hits = msub.sum(axis=1)
    col_hits = msub.sum(axis=0)
    if row_hits.max() < 2 or col_hits.max() < 2:
        return None
    rows = np.where(row_hits >= max(2.0, 0.5 * row_hits.max()))[0]
    cols = np.where(col_hits >= max(2.0, 0.5 * col_hits.max()))[0]
    if rows.size < 2 or cols.size < 2:
        return None

    ex = _cross_edges(vsub[rows].mean(axis=0), int(cols[0]), int(cols[-1]))
    ey = _cross_edges(vsub[:, cols].mean(axis=1), int(rows[0]), int(rows[-1]))
    if ex is None or ey is None:
        return None

    w_sub, h_sub = ex[1] - ex[0], ey[1] - ey[0]
    # 合理性校验，防极端背景下把整片区域当灯条
    if not (0.5 * w <= w_sub <= 1.8 * w) or not (0.5 * h <= h_sub <= 1.8 * h):
        return None
    if w_sub < 2.0 or h_sub < 4.0:
        return None
    return (x0 + (ex[0] + ex[1]) / 2.0, y0 + (ey[0] + ey[1]) / 2.0, w_sub, h_sub)


def _subpix_quad(v, mask, lights):
    """两根灯条 → 装甲板亚像素四角（直接用灯条自身的亚像素宽高，带倾斜）

    返回的四个角点顺序：左上 → 右上 → 右下 → 左下，与 solver.OBJ_PTS 对应。
    与旧方案的区别：不再强行用「宽度 × 板宽高比」重建高度，而是用两根灯条各自的
    亚像素宽高，避免灯条高度与装甲板高度不一致带来的系统性距离偏差。
    """
    if not lights or len(lights) != 2:
        return None
    a = _lamp_subpix(v, mask, lights[0])
    b = _lamp_subpix(v, mask, lights[1])
    if a is None or b is None:
        return None
    lcx, lcy, lw, lh = a
    rcx, rcy, rw, rh = b
    xl, xr = lcx - lw / 2.0, rcx + rw / 2.0
    w_rect = (lights[1][0] + lights[1][2]) - lights[0][0]
    if not (0.7 * w_rect <= xr - xl <= 1.3 * w_rect):
        return None
    # 每根灯条用自己的高度（左灯控制左边两点，右灯控制右边两点）
    return {
        "xl": float(xl), "xr": float(xr),
        "ycl": float(lcy), "ycr": float(rcy),
        "hl": float(lh), "hr": float(rh),
        "corners": np.array([
            [xl, lcy - lh / 2.0], [xr, rcy - rh / 2.0],
            [xr, rcy + rh / 2.0], [xl, lcy + lh / 2.0],
        ], dtype=np.float64),
    }


def _pair(img, lights, p):
    cand = []
    for i in range(len(lights)):
        for j in range(i + 1, len(lights)):
            x1, y1, w1, h1 = lights[i]
            x2, y2, w2, h2 = lights[j]
            if x1 > x2:
                x1, y1, w1, h1, x2, y2, w2, h2 = x2, y2, w2, h2, x1, y1, w1, h1
            hm = (h1 + h2) / 2
            dh = abs(h1 - h2) / max(hm, 1)
            dy = abs((y1 + h1 / 2) - (y2 + h2 / 2)) / max(hm, 1)
            dx = x2 - (x1 + w1)
            ratio = dx / max(hm, 1)
            if dh > p.max_dh or dy > p.max_dy:
                continue
            if not (p.gap_ratio[0] <= ratio <= p.gap_ratio[1]):
                continue
            score_digit = _digit_score(img, x1 + w1, x2, min(y1, y2),
                                       max(y1 + h1, y2 + h2))
            if score_digit < p.min_digit:
                continue
            score = score_digit * 2 - dh - dy * 0.5
            cand.append((score, i, j, score_digit))
    cand.sort(reverse=True)
    used, armors = set(), []
    for score, i, j, sd in cand:
        if i in used or j in used:
            continue
        used |= {i, j}
        x1, y1, w1, h1 = lights[i]
        x2, y2, w2, h2 = lights[j]
        armors.append({
            "corners": [[x1, y1], [x2 + w2, y2], [x2 + w2, y2 + h2], [x1, y1 + h1]],
            "center": [(x1 + x2 + w2) / 2, (min(y1, y2) + max(y1 + h1, y2 + h2)) / 2],
            "digit_score": round(sd, 4),
            "score": round(score, 4),
            "lights": [lights[i], lights[j]],
        })
        if len(armors) >= p.max_armors:
            break
    return armors


def detect_image(img_bgr, params=None, subpixel=True):
    """输入 BGR 图，返回 {"lights": {"blue": [...], "red": [...]}, "armors": [...]}

    subpixel=True 时对每块装甲板做灯条亚像素边缘拟合，结果放在 a["subpix"]
    （键 xl/xr/ycl/ycr/corners）。后续 PnP 用 a["subpix"]["corners"] 精度更高；
    拟合失败则不影响原始 corners，自动退化为整像素方案。
    """
    p = params or DetectorParams()
    masks = _masks(img_bgr, p)
    v = (cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)[:, :, 2].astype(np.float32)
         if subpixel else None)
    all_armors, all_lights = [], {}
    for color, mask in masks.items():
        ls = _lights(mask, p)
        all_lights[color] = ls
        for a in _pair(img_bgr, ls, p):
            a["color"] = color
            if v is not None:
                q = _subpix_quad(v, mask, a["lights"])
                if q is not None:
                    a["subpix"] = q
            all_armors.append(a)
    all_armors.sort(key=lambda a: a["score"], reverse=True)
    return {"lights": all_lights, "armors": all_armors}


def draw_debug(img_bgr, result, params=None):
    """把灯条 / 装甲板 / 统计画到图上，返回新图"""
    p = params or DetectorParams()
    d = img_bgr.copy()
    for color, ls in result["lights"].items():
        for (x, y, w, h) in ls:
            cv2.rectangle(d, (x, y), (x + w, y + h), (0, 255, 0), 2)
    for a in result["armors"]:
        color = (0, 255, 255) if a["color"] == "blue" else (0, 0, 255)
        if a.get("subpix") is not None:
            pts = np.round(a["subpix"]["corners"]).astype(np.int32)
        else:
            pts = np.array(a["corners"], np.int32)
        cv2.polylines(d, [pts], True, color, 2)
        if a.get("subpix") is not None:   # 亚像素结果用细线再画一遍（虚线对比）
            cv2.polylines(d, [np.array(a["corners"], np.int32)], True, (0, 0, 255), 1)
        cx, cy = int(a["center"][0]), int(a["center"][1])
        cv2.putText(d, f'{a["color"]} d={a["digit_score"]:.2f}', (cx - 60, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(d, f'armors={len(result["armors"])}  lights={sum(len(v) for v in result["lights"].values())}',
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    return d
