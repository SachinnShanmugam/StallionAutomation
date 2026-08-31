import sys
import subprocess

try:
    import fitz
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pymupdf"])
    import fitz

def read_pdf(pdf_path):
    print(f"\n==================================================")
    print(f" READING: {pdf_path}")
    print(f"==================================================")
    doc = fitz.open(pdf_path)
    for i, page in enumerate(doc):
        print(f"\n--- PAGE {i+1} ---")
        print(page.get_text())

read_pdf('/mnt/c/Users/SACHIN/Stallion/Stallion_Design/STALLION-VTOL-FILES-atx12o/STALLION VTOL FILES/V1/STALLION VTOL MANUAL V1.pdf')
read_pdf('/mnt/c/Users/SACHIN/Stallion/Stallion_Design/STALLION-VTOL-FILES-atx12o/STALLION VTOL FILES/V2/STALLION VTOL SAMPLE WIRING DIAGRAM.pdf')
