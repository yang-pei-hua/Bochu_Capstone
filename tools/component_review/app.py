"""A small native desktop UI for reviewing exported component contact sheets."""
from pathlib import Path
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .store import LABELS
from .viewer import ImageViewer


SHORT_LABELS = {"pending": "未审核", "problem": "有问题", "correct": "划分正确", "deferred": "暂不决定"}
COLORS = {"pending": "#64748b", "problem": "#bb3541", "correct": "#18794e", "deferred": "#9a650d"}


class ReviewApp:
    def __init__(self, root, store, lock=None):
        self.root, self.store, self.dataset, self.lock = root, store, store.dataset, lock
        self.current = None
        self.visible = []
        self._changing_list = False
        self.mode = "contact"
        root.title("点云划分审核 · 本地保存")
        width, height = min(1450, root.winfo_screenwidth()-80), min(940, root.winfo_screenheight()-100)
        root.geometry(f"{width}x{height}+35+35");root.minsize(min(1100,width), min(700,height))
        root.configure(background="#f4f6f9")
        root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("TLabel", font=("Microsoft YaHei UI", 10))
        style.configure("TButton", font=("Microsoft YaHei UI", 10), padding=(9, 5))
        style.configure("Treeview", font=("Microsoft YaHei UI", 10), rowheight=28)
        style.configure("Treeview.Heading", font=("Microsoft YaHei UI", 10, "bold"))
        outer = ttk.Frame(root, padding=14);outer.pack(fill="both", expand=True)
        header = ttk.Frame(outer);header.pack(fill="x", pady=(0, 9))
        ttk.Label(header, text="点云划分审核", font=("Microsoft YaHei UI", 20, "bold")).pack(side="left")
        ttk.Button(header, text="打开审核记录目录", command=lambda: self.open_folder(store.directory)).pack(side="right")
        ttk.Button(header, text="切换检查包…", command=self.choose_dataset).pack(side="right", padx=8)
        self.progress = tk.StringVar()
        ttk.Label(outer, textvariable=self.progress, foreground="#536479").pack(anchor="w", pady=(0, 10))
        content = ttk.Panedwindow(outer, orient="horizontal");content.pack(fill="both", expand=True)
        sidebar = ttk.Frame(content, padding=(0, 0, 12, 0));content.add(sidebar, weight=0)
        self.parent_filter = tk.StringVar(value="全部工件")
        parent_combo = ttk.Combobox(sidebar, state="readonly", textvariable=self.parent_filter,
                                    values=["全部工件"]+self.dataset.parents, width=29)
        parent_combo.pack(fill="x", pady=(0, 7));parent_combo.bind("<<ComboboxSelected>>", self.change_filter)
        self.status_filter = tk.StringVar(value="全部状态")
        status_combo = ttk.Combobox(sidebar, state="readonly", textvariable=self.status_filter,
            values=["全部状态", "未审核", "有问题", "划分正确", "暂不决定"], width=29)
        status_combo.pack(fill="x", pady=(0, 7));status_combo.bind("<<ComboboxSelected>>", self.change_filter)
        self.search = tk.StringVar()
        search = ttk.Entry(sidebar, textvariable=self.search, width=29)
        search.pack(fill="x");search.bind("<Return>", self.change_filter)
        ttk.Label(sidebar, text="输入编号或名称后按 Enter 筛选", foreground="#64748b").pack(anchor="w", pady=(3, 7))
        list_frame = ttk.Frame(sidebar);list_frame.pack(fill="both", expand=True)
        self.list = ttk.Treeview(list_frame, columns=("name", "status"), show="headings", selectmode="browse", height=20)
        self.list.heading("name", text="点云编号");self.list.heading("status", text="判断")
        self.list.column("name", width=186, minwidth=130);self.list.column("status", width=72, minwidth=62, stretch=False)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.list.yview)
        self.list.configure(yscrollcommand=scrollbar.set)
        self.list.pack(side="left", fill="both", expand=True);scrollbar.pack(side="right", fill="y")
        for key, color in COLORS.items():
            self.list.tag_configure(key, foreground=color)
        self.list.bind("<<TreeviewSelect>>", self.list_selected)
        main = ttk.Frame(content);content.add(main, weight=1)
        self.title = tk.StringVar();self.details = tk.StringVar()
        ttk.Label(main, textvariable=self.title, font=("Microsoft YaHei UI", 16, "bold")).pack(anchor="w")
        self.detail_label = ttk.Label(main, textvariable=self.details, foreground="#64748b")
        self.detail_label.pack(anchor="w", pady=(4, 7))
        controls = ttk.Frame(main);controls.pack(fill="x", pady=(0, 7))
        ttk.Button(controls, text="上一张  ←", command=lambda: self.navigate(-1)).pack(side="left")
        ttk.Button(controls, text="下一张  →", command=lambda: self.navigate(1)).pack(side="left", padx=5)
        ttk.Button(controls, text="下个未审核", command=self.next_pending).pack(side="left")
        ttk.Button(controls, text="拼图 / 整件定位图", command=self.toggle_locator).pack(side="left", padx=8)
        ttk.Button(controls, text="点云文件夹", command=self.open_current).pack(side="left")
        self.zoom_text = tk.StringVar(value="100%")
        ttk.Label(controls, textvariable=self.zoom_text, width=6, anchor="e").pack(side="right")
        self.viewer = ImageViewer(main, lambda scale: self.zoom_text.set(f"{round(scale*100)}%"))
        ttk.Button(controls, text="适应窗口", command=self.viewer.fit).pack(side="right", padx=4)
        ttk.Button(controls, text="100%", command=self.viewer.actual_size).pack(side="right")
        self.viewer.pack(fill="both", expand=True)
        notes = ttk.Frame(main);notes.pack(fill="x", pady=(9, 7))
        ttk.Label(notes, text="备注（可选）").pack(side="left")
        self.note = tk.StringVar()
        self.note_entry = ttk.Entry(notes, textvariable=self.note)
        self.note_entry.pack(side="left", fill="x", expand=True, padx=(9, 0))
        buttons = ttk.Frame(main);buttons.pack(fill="x")
        self.problem_button = tk.Button(buttons, text=LABELS["problem"], command=lambda: self.decide("problem"),
            background="#bd3c49", activebackground="#a62f3c", foreground="white", activeforeground="white",
            font=("Microsoft YaHei UI", 11, "bold"), relief="flat", padx=12, pady=13, cursor="hand2")
        self.correct_button = tk.Button(buttons, text=LABELS["correct"], command=lambda: self.decide("correct"),
            background="#21835e", activebackground="#176c4c", foreground="white", activeforeground="white",
            font=("Microsoft YaHei UI", 11, "bold"), relief="flat", padx=12, pady=13, cursor="hand2")
        self.defer_button = tk.Button(buttons, text=LABELS["deferred"], command=lambda: self.decide("deferred"),
            background="#b17a22", activebackground="#926018", foreground="white", activeforeground="white",
            font=("Microsoft YaHei UI", 11, "bold"), relief="flat", padx=12, pady=13, cursor="hand2")
        for column, button in enumerate((self.problem_button, self.correct_button, self.defer_button)):
            button.grid(row=0, column=column, sticky="ew", padx=(0, 7 if column<2 else 0))
            buttons.columnconfigure(column, weight=2 if column==1 else 1)
        ttk.Label(main, text="1 有问题 · 2 正确 · 3 暂不决定 · 点击判断后自动下一张 · 滚轮缩放 / 拖动查看",
                  foreground="#64748b").pack(anchor="w", pady=(7, 0))
        self.message = tk.StringVar(value=f"判断实时保存到：{store.path}")
        ttk.Label(outer, textvariable=self.message, foreground="#536479", wraplength=1350).pack(fill="x", pady=(10, 0))
        for key, decision in (("1", "problem"), ("2", "correct"), ("3", "deferred")):
            root.bind(key, lambda event, value=decision: self.shortcut(event, lambda: self.decide(value)))
        root.bind("<Left>", lambda event: self.shortcut(event, lambda: self.navigate(-1)))
        root.bind("<Right>", lambda event: self.shortcut(event, lambda: self.navigate(1)))
        self.refresh_list()
        cursor = store.data.get("cursor")
        target = self.dataset.by_name.get(cursor) or next((i for i in self.visible if store.decision(i)=="pending"), self.visible[0])
        self.show(target)

    def shortcut(self, event, action):
        if event.widget.winfo_class() not in ("Entry", "TEntry", "Text", "TCombobox"):
            action();return "break"

    def refresh_list(self):
        self._changing_list = True
        current = self.current.name if self.current else None
        self.list.delete(*self.list.get_children())
        self.visible = [item for item in self.dataset.items
            if (self.parent_filter.get()=="全部工件" or item.parent==self.parent_filter.get())
            and (self.status_filter.get()=="全部状态" or SHORT_LABELS[self.store.decision(item)]==self.status_filter.get())
            and self.search.get().strip().lower() in item.name.lower()]
        for item in self.visible:
            decision = self.store.decision(item)
            self.list.insert("", "end", iid=item.name, values=(item.name, SHORT_LABELS[decision]), tags=(decision,))
        if current and self.list.exists(current):
            self.list.selection_set(current);self.list.see(current)
        counts = self.store.counts()
        reviewed = len(self.dataset.items)-counts["pending"]
        self.progress.set(f"已处理 {reviewed}/{len(self.dataset.items)}  ·  正确 {counts['correct']}  ·  有问题 {counts['problem']}  ·  暂不决定 {counts['deferred']}  ·  未审核 {counts['pending']}")
        self._changing_list = False

    def save_navigation(self, cursor=None):
        if self.current is None:
            return True
        try:
            self.store.checkpoint(self.current, self.note.get(), cursor or self.current.name)
            return True
        except (OSError, ValueError) as error:
            messagebox.showerror("审核记录未保存", str(error), parent=self.root)
            return False

    def show(self, item, save=False):
        if save and not self.save_navigation(item.name):
            return
        self.current = item;self.mode = "contact"
        self.note.set(self.store.note(item))
        self.title.set(item.name)
        decision = self.store.decision(item)
        index = self.dataset.items.index(item)+1
        kind = "未归属点集合" if item.number==0 else ("算法已闭合" if item.closed else "算法未闭合")
        self.details.set(f"第 {index}/{len(self.dataset.items)} 张  ·  {item.points:,} 点  ·  {kind}  ·  {LABELS[decision]}")
        self.detail_label.configure(foreground=COLORS[decision])
        try:
            self.viewer.load(item.image)
            for button in (self.problem_button, self.correct_button, self.defer_button):button.configure(state="normal")
            if not item.current:
                self.correct_button.configure(state="disabled")
                self.message.set("此点云已改变，与拼图来源不一致；请重新生成图片后再纳入测试集。")
        except OSError as error:
            self.viewer.clear(str(error))
            for button in (self.problem_button, self.correct_button, self.defer_button):button.configure(state="disabled")
        self._changing_list = True
        if self.list.exists(item.name):
            self.list.selection_set(item.name);self.list.see(item.name)
        self._changing_list = False

    def list_selected(self, _event=None):
        if self._changing_list:
            return
        selected = self.list.selection()
        if selected and (self.current is None or selected[0]!=self.current.name):
            self.show(self.dataset.by_name[selected[0]], save=True)

    def change_filter(self, _event=None):
        if not self.save_navigation():return
        old = self.current
        self.refresh_list()
        if old in self.visible:self.show(old)
        elif self.visible:self.show(self.visible[0])
        else:
            self.current = None;self.title.set("当前筛选下没有点云");self.details.set("");self.note.set("")
            self.viewer.clear()
            for button in (self.problem_button, self.correct_button, self.defer_button):button.configure(state="disabled")

    def navigate(self, step):
        if not self.visible:return
        index = self.visible.index(self.current) if self.current in self.visible else 0
        target = self.visible[max(0, min(len(self.visible)-1, index+step))]
        self.show(target, save=True)

    def next_pending(self):
        if not self.visible:return
        index = self.visible.index(self.current) if self.current in self.visible else -1
        order = self.visible[index+1:]+self.visible[:index+1]
        target = next((i for i in order if self.store.decision(i)=="pending"), None)
        if target:self.show(target, save=True)
        else:self.message.set("当前筛选范围没有未审核图片，可回看或切换筛选。")

    def decide(self, decision):
        if not self.current:return
        old = self.current
        index = self.visible.index(old) if old in self.visible else 0
        following = self.visible[index+1] if index+1<len(self.visible) else None
        try:
            self.store.decide(old, decision, self.note.get(), following.name if following else old.name)
        except (OSError, ValueError) as error:
            messagebox.showerror("审核记录未保存", str(error), parent=self.root);return
        self.refresh_list()
        target = following if following in self.visible else (self.visible[min(index,len(self.visible)-1)] if self.visible else None)
        if target:self.show(target)
        else:
            self.current=None;self.title.set("当前范围已全部处理");self.details.set("");self.note.set("");self.viewer.clear("当前筛选已完成")
            for button in (self.problem_button,self.correct_button,self.defer_button):button.configure(state="disabled")
        self.message.set(f"已保存：{old.name} → {LABELS[decision]}"+("。已到当前范围最后一张。" if following is None else ""))
        if self.store.last_export_error:
            self.message.set("判断已保存在 JSON；CSV/测试集清单暂未更新："+self.store.last_export_error)

    def toggle_locator(self):
        if not self.current:return
        path = self.dataset.locators[self.current.parent] if self.mode=="contact" else self.current.image
        try:
            self.viewer.load(path)
            self.mode = "locator" if self.mode=="contact" else "contact"
            self.message.set("整件定位图：滚轮放大编号，拖动查看；再次点击切回拼图。" if self.mode=="locator" else f"判断实时保存到：{self.store.path}")
        except OSError as error:
            messagebox.showerror("无法打开图片", str(error), parent=self.root)

    def open_folder(self, path):
        try:
            if os.name=="nt":os.startfile(path)
            else:
                import subprocess
                subprocess.Popen(["open" if __import__("sys").platform=="darwin" else "xdg-open", str(path)])
        except OSError as error:messagebox.showerror("无法打开目录", str(error), parent=self.root)

    def open_current(self):
        if self.current:self.open_folder(self.current.directory)

    def choose_dataset(self):
        directory = filedialog.askdirectory(parent=self.root, title="选择点云检查包目录", initialdir=self.dataset.root.parent)
        if not directory or Path(directory).resolve()==self.dataset.root:return
        if not self.save_navigation():return
        # A new independent process owns the new package's lock and state.
        import subprocess, sys
        script = Path(__file__).resolve().parents[1]/"review_component_clouds.py"
        subprocess.Popen([sys.executable, str(script), "--dataset", directory])

    def close(self):
        if not self.save_navigation():return
        if self.lock:self.lock.close()
        self.root.destroy()
