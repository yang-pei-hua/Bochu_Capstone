#!/usr/bin/env python3
"""Open the local, autosaving component-cloud review desktop application."""
import argparse
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]


def enable_dpi_awareness():
    if sys.platform == "win32":
        import ctypes
        try:ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=ROOT/"outputs/component-cloud-review-20261101")
    parser.add_argument("--reviews", type=Path, help="Separate review-record directory (default: dataset/review)")
    args = parser.parse_args()
    enable_dpi_awareness()
    import tkinter as tk
    from tkinter import messagebox
    from .store import Dataset, ReviewLock, ReviewStore
    from .app import ReviewApp
    root = tk.Tk();root.withdraw()
    lock = None
    try:
        dataset = Dataset(args.dataset)
        directory = args.reviews or dataset.root/"review"
        lock = ReviewLock(directory)
        store = ReviewStore(dataset, directory)
        ReviewApp(root, store, lock)
        root.deiconify()
        root.mainloop()
    except Exception as error:
        messagebox.showerror("点云审核工具", str(error), parent=root)
        traceback.print_exc()
        root.destroy()
        if lock:lock.close()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
