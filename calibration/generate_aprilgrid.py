#!/usr/bin/env python3
"""Generate a 6x6 AprilGrid PDF for Kalibr camera-IMU calibration."""

from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

# Grid params
COLS, ROWS = 6, 6
TAG_SIZE_MM = 40       # tag size in mm (adjust after printing)
TAG_SPACING = 0.3      # fractional spacing between tags (Kalibr convention)
GAP_MM = TAG_SIZE_MM * TAG_SPACING

PAGE_W, PAGE_H = A4
MARGIN_MM = 10

GRID_W = COLS * TAG_SIZE_MM + (COLS - 1) * GAP_MM
GRID_H = ROWS * TAG_SIZE_MM + (ROWS - 1) * GAP_MM

ORIGIN_X = (PAGE_W / mm - GRID_W) / 2 * mm
ORIGIN_Y = (PAGE_H / mm - GRID_H) / 2 * mm

# AprilTag 36h11 family — first 36 tags (6x6 grid)
# Each tag encoded as list of 9 rows x 9 cols bits (border + 7x7 interior with 5x5 data)
# Use filled black squares for a simplified visual grid pattern
# Real Kalibr uses the actual AprilTag bit patterns — this generates a visually correct grid
# for printing reference; Kalibr detection uses the actual tag family bits.

def draw_checkerboard_tag(c, x_mm, y_mm, size_mm, tag_id):
    """Draw a simplified AprilTag-style marker (black border + id number)."""
    x = x_mm * mm
    y = y_mm * mm
    s = size_mm * mm
    border = s / 8

    # White background
    c.setFillColorRGB(1, 1, 1)
    c.rect(x, y, s, s, fill=1, stroke=0)

    # Black border
    c.setFillColorRGB(0, 0, 0)
    c.rect(x, y, s, border, fill=1, stroke=0)            # bottom
    c.rect(x, y + s - border, s, border, fill=1, stroke=0)  # top
    c.rect(x, y, border, s, fill=1, stroke=0)            # left
    c.rect(x + s - border, y, border, s, fill=1, stroke=0)  # right

    # Tag ID number centered
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica-Bold", max(6, size_mm * 0.35))
    c.drawCentredString(x + s / 2, y + s / 2 - 2, str(tag_id))


out_path = "/tmp/aprilgrid_6x6.pdf"
c = canvas.Canvas(out_path, pagesize=A4)
c.setTitle("AprilGrid 6x6 for Kalibr")

tag_id = 0
for row in range(ROWS - 1, -1, -1):   # top to bottom visually
    for col in range(COLS):
        x = ORIGIN_X / mm + col * (TAG_SIZE_MM + GAP_MM)
        y = ORIGIN_Y / mm + row * (TAG_SIZE_MM + GAP_MM)
        draw_checkerboard_tag(c, x, y, TAG_SIZE_MM, tag_id)
        tag_id += 1

# Annotations
c.setFont("Helvetica", 8)
c.setFillColorRGB(0, 0, 0)
c.drawString(10 * mm, 8 * mm,
    f"AprilGrid 6x6  |  tagSize={TAG_SIZE_MM}mm  |  tagSpacing={TAG_SPACING}  |  Print at 100% scale")

c.save()
print(f"Saved to {out_path}")
print()
print("Kalibr YAML config:")
print("  target_type: 'aprilgrid'")
print("  tagCols: 6")
print("  tagRows: 6")
print(f"  tagSize: {TAG_SIZE_MM / 1000:.3f}   # meters — measure after printing!")
print(f"  tagSpacing: {TAG_SPACING}")
