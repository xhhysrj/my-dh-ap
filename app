# -*- coding: utf-8 -*-
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.widget import Widget
from kivy.graphics import Color, Line, Ellipse

# ─── A.  顶点在 ASCII 网格中的“行-列”布置
Graph = [
    ['10',' '*10],
    ['8',' ','9',' '*8],
    ['6',' ','7',' ','16','17',' '*5],
    ['0','4',' ','5','14','15','21','22','25','28','31'],
    [' ','3',' ',' ',' ','13',' ','20','24','27','30'],
    [' ','1',' ','2','11','12','18','19','23','26','29']
]

# 建立 {顶点: (col,row)} 查表
NODE_POS = {}
for r, row in enumerate(Graph):
    for c, cell in enumerate(row):
        if cell.strip():               # 非空格即是一个顶点
            NODE_POS[int(cell)] = (c, r)
# ─── B.  原来的距离表 + Floyd–Warshall
INF = float("inf")
Edge = [
    [127,185,42,47,118,117,146,187,191,288,241,284,206,217,206,236,293,352,365,368,305,361,550,540,500,642,602,562,682,642,602],
    [60,5,56,98,126,126,196,200,297,116,159,137,198,173,206,272,229,274,275,254,268,350,420,460,429,481,521,475,557,578],
    [94,156,114,204,142,274,245,375,55,98,112,173,148,181,247,168,213,250,229,283,260,330,374,360,394,434,414,454,494],
    [42,54,70,82,140,144,241,123,139,139,101,142,161,227,267,279,247,239,279,325,315,348,469,431,463,516,488,512],
    [63,53,44,113,105,166,166,186,137,97,137,110,167,255,314,288,208,248,373,363,400,456,416,436,495,465,488],
    [93,30,157,106,203,79,124,72,32,72,36,102,190,267,223,143,183,318,298,330,391,351,371,440,400,420],
    [51,68,51,117,202,266,214,128,166,119,185,318,358,313,234,273,405,389,337,494,454,434,540,498,483],
    [71,42,129,149,183,151,81,104,57,123,275,290,250,171,210,336,326,274,430,391,371,475,435,420],
    [78,152,281,351,265,198,236,151,245,412,445,379,301,337,450,440,399,543,503,483,590,550,532],
    [126,176,248,174,94,131,50,144,284,344,278,199,238,364,354,315,440,398,382,491,451,431],
    [332,366,316,249,287,202,290,436,477,409,352,369,589,476,411,570,530,495,604,564,544],
    [24,56,105,92,145,206,91,131,164,159,206,170,232,272,306,370,417,355,428,461],
    [9,91,71,108,131,64,108,112,81,196,220,290,330,265,292,343,331,385,418],
    [62,36,39,107,79,142,117,131,188,225,204,254,299,340,360,355,416,448],
    [44,13,91,172,222,188,105,165,354,344,297,373,360,320,427,450,410],
    [41,33,101,152,127,76,133,230,220,188,298,295,265,347,406,358],
    [41,221,273,243,106,175,320,290,240,386,346,316,508,444,424],
    [156,207,174,41,111,240,230,200,362,320,257,427,390,360],
    [60,100,140,140,90,130,170,180,220,260,240,270,310],
    [100,140,140,40,100,140,120,160,200,180,220,260],
    [100,110,70,60,100,160,120,160,220,180,225],
    [57,197,157,117,267,187,147,287,257,207],
    [140,100,60,210,130,90,230,190,150],
    [70,120,100,140,180,160,190,230],
    [100,120,80,123,148,130,170],
    [156,136,80,276,226,146],
    [100,140,60,105,150],
    [100,105,60,110],
    [140,100,60],
    [100,140],
    [100]
]

def metric_closure():
    n = len(Edge) + 1
    d = [[INF]*n for _ in range(n)]
    for i in range(n):
        d[i][i] = 0
    for i, row in enumerate(Edge):
        for j, w in enumerate(row):
            v = i + j + 1
            d[i][v] = d[v][i] = w
    for k in range(n):
        dk = d[k]
        for i in range(n):
            dik = d[i][k]
            if dik == INF:
                continue
            di = d[i]
            for j in range(n):
                alt = dik + dk[j]
                if alt < di[j]:
                    di[j] = alt
    return d

DIST = metric_closure()

# ─── C.  OR‑Tools 精确求解
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def solve_exact(start, stops, end):
    nodes = [start] + stops + [end]
    size  = len(nodes)
    sub   = [[DIST[u][v] for v in nodes] for u in nodes]

    man = pywrapcp.RoutingIndexManager(size, 1, [0], [size - 1])
    rout = pywrapcp.RoutingModel(man)
    cb   = rout.RegisterTransitCallback(
              lambda i, j: sub[man.IndexToNode(i)][man.IndexToNode(j)])
    rout.SetArcCostEvaluatorOfAllVehicles(cb)
    rout.AddDimension(cb, 0, 10**7, True, "Dist")

    sp = pywrapcp.DefaultRoutingSearchParameters()
    sp.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    sp.time_limit.FromSeconds(2)
    sol = rout.SolveWithParameters(sp)
    if sol is None:
        raise RuntimeError("Solver failed — try longer time limit")

    idx, path, dist = rout.Start(0), [], 0
    while not rout.IsEnd(idx):
        n   = man.IndexToNode(idx)
        path.append(nodes[n])
        nxt = sol.Value(rout.NextVar(idx))
        dist += sub[n][man.IndexToNode(nxt)]
        idx  = nxt
    path.append(nodes[man.IndexToNode(idx)])
    return dist, path

# ─── 在文件任意位置（如 solve_exact() 下方）加入一个帮助函数
def format_path(p):
    """把顶点序列转成带 Start/End 标签的字符串"""
    if not p:
        return ""
    parts = [f"{p[0]}(Start)"]                 # 起点
    parts += map(str, p[1:-1])                 # 中间顶点
    parts.append(f"{p[-1]}(End)")              # 终点
    return " -> ".join(parts)

# ─── D.  Path 可视化部件 ───
CELL   = 50          # 网格单元大小（像素）
MARGIN = 30          # 边距
RADIUS = 8           # 圆圈半径

def grid_to_xy(col, row):
    """把 (列,行) &rarr; 窗口坐标系 (x,y)；y 轴向上。"""
    x = MARGIN + col * CELL
    y = MARGIN + (len(Graph)-1 - row) * CELL
    return x, y

from kivy.core.text import Label as CoreLabel
from kivy.graphics import Color, Line, Ellipse, Rectangle

class PathWidget(Widget):
    """负责在 canvas 上画出整张地图、路径以及顶点编号。"""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.full_path = []      # 新增
        self.unique_path = []

    def set_path(self, path):
        self.full_path = path[:]             # 保留完整路径，用于画线

        seen, self.unique_path = set(), []
        for v in path:                       # 依旧先到先得去重
            if v not in seen:
                self.unique_path.append(v)
                seen.add(v)

        self._update_canvas()

    def _draw_label(self, x, y, text, dy=12):
        """在 (x,y) 上方 dy 处画居中白色文字。"""
        lbl = CoreLabel(text=text, font_size=16, color=(1, 1, 1, 1))
        lbl.refresh()
        w, h = lbl.texture.size
        Rectangle(texture=lbl.texture,
                  pos=(x - w/2+15, y + dy),
                  size=(w, h))

    def _update_canvas(self):
        self.canvas.clear()
        if not self.unique_path:
            return

        with self.canvas:
            # ── A. 整张网格：白点 ──────────────────────────
            Color(1, 1, 1)                           # 白色
            for r, row in enumerate(Graph):
                for c, cell in enumerate(row):
                    x, y = grid_to_xy(c, r)
                    if cell.strip():                 # 这是一个顶点
                        v = int(cell)
                        if v in self.unique_path:
                            continue                 # 路径上的顶点稍后画
                        # 非路径顶点 &rarr; 白点
                        Ellipse(pos=(x - RADIUS/2, y - RADIUS/2),
                                size=(RADIUS, RADIUS))
                    else:
                        # 空格位置 &rarr; 更小的白点
                        Ellipse(pos=(x - RADIUS/2, y - RADIUS/2),
                                size=(RADIUS, RADIUS))

            # ── B. 画路径线 —— 用 self.full_path 而不是 unique
            Color(0, 0.5, 0.5)
            for a, b in zip(self.full_path, self.full_path[1:]):
                x1, y1 = grid_to_xy(*NODE_POS[a])
                x2, y2 = grid_to_xy(*NODE_POS[b])
                if y1 == y2 or x1 == x2:
                    Line(points=[x1, y1, x2, y2], width=2)
                else:
                    Line(points=[x1, y1, x2, y1, x2, y2], width=2)
            # ── C. 路径顶点：红点 + 白字 ─────────────────
            Color(0.9, 0.2, 0.2)                     # 红点
            for v in self.unique_path:
                x, y = grid_to_xy(*NODE_POS[v])
                Ellipse(pos=(x - RADIUS, y - RADIUS),
                        size=(2*RADIUS, 2*RADIUS))
                self._draw_label(x, y, str(v))       # 序号标签


# ─── E.  Kivy UI ───
class CourierApp(App):
    title = "DongHuaHaJiMi"  # 添加这行代码
    def build(self):
        root = BoxLayout(orientation="vertical", padding=10, spacing=10)

        # 输入区域
        self.start_box = self._row(root, "Start (0‑31):")
        self.stops_box = self._row(root, "Stops (ps: 6 7 8):")
        self.end_box   = self._row(root, "End   (0‑31):")

        btn = Button(text="GoGoGo", size_hint_y=None, height=60)
        btn.bind(on_press=self.compute)
        root.add_widget(btn)

        self.out = Label(text="Result will appear here",
                         halign="left", valign="top",
                         size_hint_y=None, height=160)
        self.out.bind(size=self.out.setter("text_size"))
        root.add_widget(self.out)

        # —— 新增：Path(reversed) —— #
        self.out_rev = Label(text="Path(reversed) will appear here",
                             halign="left", valign="top",
                             size_hint_y=None, height=80)
        self.out_rev.bind(size=self.out_rev.setter("text_size"))
        root.add_widget(self.out_rev)

        # 可视化区域
        self.path_widget = PathWidget(size_hint_y=1)  # 余下空间全部给画布
        root.add_widget(self.path_widget)
        return root

    @staticmethod
    def _row(parent, caption):
        row = BoxLayout(size_hint_y=None, height=50)
        row.add_widget(Label(text=caption, size_hint_x=.45))
        ti = TextInput(multiline=False)
        row.add_widget(ti)
        parent.add_widget(row)
        return ti

    def compute(self, *_):
        try:
            s = int(self.start_box.text)
            e = int(self.end_box.text)
            if not 0 <= s <= 31 or not 0 <= e <= 31:
                raise ValueError("Start / End must be 0‑31.")

            stops_raw = self.stops_box.text.strip()
            stops = [int(x) for x in stops_raw.split()] if stops_raw else []
            if any(t < 0 or t > 31 for t in stops):
                raise ValueError("Stops must be 0‑31.")
            if len(stops) > 25:
                raise ValueError("Max 25 stops.")

            # ─── modify CourierApp.compute(...) 仅 2 行 ───
            dist, path = solve_exact(s, stops, e)

            self.out.text = (
                f"Shortest distance: {dist}\n\nPath:\n{format_path(path)}"
            )

            # 逆向路径输出
            self.out_rev.text = "Path(reversed):\n" + format_path(path[::-1])
            # ◎ 更新可视化
            self.path_widget.set_path(path)

        except Exception as ex:
            self.out.text = f"Error: {ex}"
            self.out_rev.text = ""

# ─── F.  入口 ───
if __name__ == "__main__":
    CourierApp().run()
