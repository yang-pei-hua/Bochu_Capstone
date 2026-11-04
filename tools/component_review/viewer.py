"""Zoomable local-image canvas; renders only the visible crop at high zoom."""
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk


class ImageViewer(ttk.Frame):
    def __init__(self, parent, on_zoom=None):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, background="#e8edf3", highlightthickness=0, cursor="fleur")
        self.horizontal = ttk.Scrollbar(self, orient="horizontal", command=self.canvas.xview)
        self.vertical = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=self._xscroll, yscrollcommand=self._yscroll)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.vertical.grid(row=0, column=1, sticky="ns")
        self.horizontal.grid(row=1, column=0, sticky="ew")
        self.rowconfigure(0, weight=1);self.columnconfigure(0, weight=1)
        self.original = None
        self.photo = None
        self.scale = 1.
        self.fit_mode = True
        self.on_zoom = on_zoom or (lambda _: None)
        self._paint_job = None
        self._region = None
        self._image_id = self.canvas.create_image(0, 0, anchor="nw")
        self._message_id = self.canvas.create_text(20, 20, anchor="nw", fill="#536476", font=("Microsoft YaHei UI", 14))
        self.canvas.bind("<Configure>", self._resize)
        self.canvas.bind("<MouseWheel>", self._wheel)
        self.canvas.bind("<Button-4>", lambda event: self.zoom(1.2, event.x, event.y))
        self.canvas.bind("<Button-5>", lambda event: self.zoom(1/1.2, event.x, event.y))
        self.canvas.bind("<ButtonPress-1>", lambda event: self.canvas.scan_mark(event.x, event.y))
        self.canvas.bind("<B1-Motion>", lambda event: self.canvas.scan_dragto(event.x, event.y, gain=1))

    def _xscroll(self, first, last):
        self.horizontal.set(first, last);self._schedule()

    def _yscroll(self, first, last):
        self.vertical.set(first, last);self._schedule()

    def _resize(self, _event=None):
        if self.fit_mode:
            self.fit()
        else:
            self._schedule()

    def _schedule(self):
        if self._paint_job is None:
            self._paint_job = self.after_idle(self._paint)

    def load(self, path):
        with Image.open(Path(path)) as image:
            self.original = image.convert("RGB")
        self.canvas.itemconfigure(self._message_id, text="")
        self.fit()

    def clear(self, message="当前筛选下没有图片"):
        self.original = None;self.photo = None
        self.canvas.itemconfigure(self._image_id, image="")
        self.canvas.itemconfigure(self._message_id, text=message)
        self._set_region(1, 1)

    def _set_region(self, width, height):
        region = (0, 0, width, height)
        if self._region != region:
            self._region = region
            self.canvas.configure(scrollregion=region)

    def fit(self):
        self.fit_mode = True
        if self.original is not None:
            width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
            self.scale = min((width-12)/self.original.width, (height-12)/self.original.height)
            self.scale = max(.01, min(self.scale, 4.))
            self.canvas.xview_moveto(0);self.canvas.yview_moveto(0)
            self._schedule()

    def actual_size(self):
        if self.original is not None:
            self.zoom(1/self.scale)

    def _geometry(self):
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        scaled_w, scaled_h = self.original.width*self.scale, self.original.height*self.scale
        return width, height, max(0, (width-scaled_w)/2), max(0, (height-scaled_h)/2), max(width, scaled_w), max(height, scaled_h)

    def zoom(self, factor, x=None, y=None):
        if self.original is None:
            return
        width, height, pad_x, pad_y, _, _ = self._geometry()
        x = width/2 if x is None else x
        y = height/2 if y is None else y
        source_x = (self.canvas.canvasx(x)-pad_x)/self.scale
        source_y = (self.canvas.canvasy(y)-pad_y)/self.scale
        self.scale = max(.02, min(8., self.scale*factor))
        self.fit_mode = False
        _, _, pad_x, pad_y, virtual_w, virtual_h = self._geometry()
        self._set_region(virtual_w, virtual_h)
        self.canvas.xview_moveto(max(0, (source_x*self.scale+pad_x-x)/virtual_w))
        self.canvas.yview_moveto(max(0, (source_y*self.scale+pad_y-y)/virtual_h))
        self._schedule()

    def _wheel(self, event):
        self.zoom(1.2 if event.delta > 0 else 1/1.2, event.x, event.y)
        return "break"

    def _paint(self):
        self._paint_job = None
        if self.original is None:
            return
        width, height, pad_x, pad_y, virtual_w, virtual_h = self._geometry()
        self._set_region(virtual_w, virtual_h)
        viewport_x, viewport_y = self.canvas.canvasx(0), self.canvas.canvasy(0)
        left, top = max(0, (viewport_x-pad_x)/self.scale), max(0, (viewport_y-pad_y)/self.scale)
        right = min(self.original.width, (viewport_x+width-pad_x)/self.scale)
        bottom = min(self.original.height, (viewport_y+height-pad_y)/self.scale)
        if right <= left or bottom <= top:
            return
        crop = self.original.crop((int(left), int(top), min(self.original.width, int(right)+1), min(self.original.height, int(bottom)+1)))
        rendered = crop.resize((max(1, round(crop.width*self.scale)), max(1, round(crop.height*self.scale))),
                               Image.Resampling.LANCZOS if self.scale < 1 else Image.Resampling.BICUBIC)
        self.photo = ImageTk.PhotoImage(rendered)
        self.canvas.coords(self._image_id, int(left)*self.scale+pad_x, int(top)*self.scale+pad_y)
        self.canvas.itemconfigure(self._image_id, image=self.photo)
        self.on_zoom(self.scale)

    def destroy(self):
        if self._paint_job is not None:
            self.after_cancel(self._paint_job)
        super().destroy()
