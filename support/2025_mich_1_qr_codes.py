import qrcode # pip install "qrcode[pil]"
from fpdf import FPDF, Align # pip install fpdf2

# Possible drop-off points
racks = ["A", "B"]
levels = ["Upper", "Lower"]
bays = [i for i in range (1,7)]

# Formatting assumptions
assert len(bays) == 6
assert len(racks) * len(levels) == 4

# Create PDF with QR Codes
pdf = FPDF(orientation='P', unit='mm', format='A4')
pdf.add_page()
pdf.set_font("Helvetica", size=6)

# Layout - all mm
qrcode_dim = 40
x_margin = 15
y_margin = 15
x_inc = qrcode_dim + 5
assert x_inc * 4 + x_margin * 2 <= 210
y_inc_qrcode = qrcode_dim + 2.5
y_inc_text = 2
y_inc = y_inc_qrcode + y_inc_text
assert y_inc * 6 + y_margin * 2 <= 297

y = y_margin

for bay in bays:
    # new row
    x = x_margin

    for rack in racks:
        for level in levels:
            msg = f"Rack {rack}, {level}, {bay}"

            # QR Code
            qr_img = qrcode.make(msg, border=0)
            pdf.image(qr_img.get_image(),
                      x=x,
                      y=y,
                      w=qrcode_dim,
                      h=qrcode_dim)
            #pdf.rect(x=x,
            #         y=y,
            #         w=qrcode_dim,
            #         h=qrcode_dim)

            # Label
            pdf.text(x, y + y_inc_qrcode, msg)

            # next column
            x += x_inc

    # next row
    y += y_inc

# Done
pdf.output(name = '2025_mich_1_qr_codes.pdf')