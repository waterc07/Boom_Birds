#!/usr/bin/env python3
"""Print the agreed A4 / tag36h11 / ID7 / 70 mm master; no camera profile changes."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

REPO=Path(__file__).resolve().parents[3]


def generate(output):
    master=REPO/"docs/assets/tag36h11-id7-black70mm-a4.svg"
    tree=ET.fromstring(master.read_text())
    if tree.get("width")!="210mm" or tree.get("height")!="297mm":
        raise ValueError("agreed_master_page_changed")
    rectangles=[r for r in tree if r.tag.endswith("rect") and r.get("fill")=="black"]
    lowX=min(float(r.get("x")) for r in rectangles)
    lowY=min(float(r.get("y")) for r in rectangles)
    highX=max(float(r.get("x"))+float(r.get("width")) for r in rectangles)
    highY=max(float(r.get("y"))+float(r.get("height")) for r in rectangles)
    if (lowX,lowY,highX,highY)!=(70.,113.5,140.,183.5):
        raise ValueError("agreed_master_black_size_changed")
    output.mkdir(parents=True,exist_ok=True)
    target=output/"tag36h11-id7-black70mm-a4.pdf"
    pdf=canvas.Canvas(str(target),pagesize=(210*mm,297*mm),pageCompression=1)
    pdf.setTitle("tag36h11 ID7 - black 70 mm - A4 actual size")
    pdf.setViewerPreference("PrintScaling","None")
    pdf.setFillColorRGB(0,0,0)
    for rect in rectangles:
        x,y,w,h=(float(rect.get(key)) for key in ("x","y","width","height"))
        pdf.rect(x*mm,(297-y-h)*mm,w*mm,h*mm,fill=1,stroke=0)
    pdf.setFont("Helvetica",13)
    pdf.drawString(20*mm,266*mm,"TAG36H11 / ID 7 / A4")
    pdf.setFont("Helvetica",10)
    pdf.drawString(20*mm,251*mm,"Black outer square: 70 x 70 mm (white margin excluded)")
    pdf.drawString(20*mm,239*mm,"Actual size 100% / no fit / no mirror / no stretch")
    pdf.drawString(20*mm,87*mm,"Keep at least 8.75 mm of white space on all four sides")
    pdf.drawString(20*mm,75*mm,"Measure both black edges and the 100 mm bar after printing")
    pdf.setLineWidth(.5)
    pdf.line(55*mm,40*mm,155*mm,40*mm)
    for index in range(11):
        x=(55+10*index)*mm
        pdf.line(x,38*mm,x,42*mm)
    pdf.setFont("Helvetica",8)
    pdf.drawString(55*mm,47*mm,"100 mm reference")
    pdf.showPage();pdf.save()
    manifest=dict(paper_mm=[210,297],family="tag36h11",id=7,black_side_mm=70.,
        white_margin_min_mm=8.75,size_m=.07,master=str(master),
        master_sha256=hashlib.sha256(master.read_bytes()).hexdigest(),
        generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        pdf_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        physical_print_measured=False,real_camera_verified=False)
    (output/"print-manifest.json").write_text(json.dumps(manifest,indent=2))
    print(target)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args()
    generate(args.out)


if __name__=="__main__":main()
