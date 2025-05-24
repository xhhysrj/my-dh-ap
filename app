
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button    import Button
from kivy.uix.label     import Label
from kivy.uix.textinput import TextInput
from kivy.uix.widget    import Widget
from kivy.core.text     import Label as CoreLabel
from kivy.graphics      import Color, Line, Ellipse, Rectangle
from collections        import Counter
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# ─── 常量与网格数据
ROWS, COLS = 12, 43
CELL   = 30
MARGIN = 30
RADIUS = 6

NODE_POS = {
    10:(1, 3),  8:(3, 3),  6:(5, 3),  0:(7, 3),
     4:(7, 7),  3:(9, 7),  1:(11,7),
     9:(3,10),  7:(5,10),  5:(7,13),  2:(11,13),
    16:(5,17), 14:(7,17), 11:(11,17),
    17:(5,21), 15:(7,21), 13:(9,21), 12:(11,21),
    21:(7,25), 18:(11,25),
    22:(7,29), 20:(9,29), 19:(11,29),
    25:(7,33), 24:(9,33), 23:(11,33),
    28:(7,37), 27:(9,37), 26:(11,37),
    31:(7,41), 30:(9,41), 29:(11,41)
}

_path_segments = [
    (( 2,  1), (12,  1)), (( 2,  5), (12,  5)), (( 7, 10), (12, 10)),
    (( 4, 15), (12, 15)), (( 7, 19), (12, 19)), (( 7, 23), (12, 23)),
    (( 7, 27), (12, 27)), (( 7, 31), (12, 31)), (( 7, 35), (12, 35)),
    (( 7, 39), (12, 39)), (( 7, 43), (12, 43)), (( 2,  2), ( 2,  4)),
    (( 4,  2), ( 4,  4)), (( 6,  2), ( 6,  4)), ((12,  2), (12,  4)),
    (( 4,  6), ( 4, 14)), (( 6,  6), ( 6, 14)), (( 8,  6), ( 8,  9)),
    ((10,  6), (10,  9)), ((12,  6), (12,  9)), (( 8, 11), ( 8, 14)),
    ((10, 11), (10, 14)), ((12, 11), (12, 14)), (( 6, 16), ( 6, 43)),
    (( 8, 16), ( 8, 18)), ((10, 16), (10, 18)), ((12, 16), (12, 18)),
    (( 8, 20), ( 8, 22)), ((10, 20), (10, 22)), ((12, 20), (12, 22)),
    (( 8, 24), ( 8, 26)), ((10, 24), (10, 26)), ((12, 24), (12, 26)),
    (( 8, 28), ( 8, 30)), ((10, 28), (10, 30)), ((12, 28), (12, 30)),
    (( 8, 32), ( 8, 34)), ((10, 32), (10, 34)), ((12, 32), (12, 34)),
    (( 8, 36), ( 8, 38)), ((10, 36), (10, 38)), ((12, 36), (12, 38)),
    (( 8, 40), ( 8, 42)), ((10, 40), (10, 42)), ((12, 40), (12, 42))
]

PATH_CELLS = set()
for (r1, c1), (r2, c2) in _path_segments:
    if r1 == r2:
        for c in range(min(c1, c2), max(c1, c2) + 1):
            PATH_CELLS.add((r1, c))
    else:
        for r in range(min(r1, r2), max(r1, r2) + 1):
            PATH_CELLS.add((r, c1))

# ─── 二、距离矩阵
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
    [100,140,60,100,150],
    [100,100,60,110],
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

# ─── 三、最短路求解
def solve_exact(start, stops, end):
    nodes = [start] + stops + [end]
    size  = len(nodes)
    sub   = [[DIST[u][v] for v in nodes] for u in nodes]

    man  = pywrapcp.RoutingIndexManager(size, 1, [0], [size - 1])
    rout = pywrapcp.RoutingModel(man)
    cb   = rout.RegisterTransitCallback(
        lambda i, j: sub[man.IndexToNode(i)][man.IndexToNode(j)]
    )
    rout.SetArcCostEvaluatorOfAllVehicles(cb)
    rout.AddDimension(cb, 0, 10**7, True, "Dist")

    sp = pywrapcp.DefaultRoutingSearchParameters()
    sp.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    sp.time_limit.FromSeconds(2)
    sol = rout.SolveWithParameters(sp)
    if sol is None:
        raise RuntimeError("Solver failed")

    idx, path, dist = rout.Start(0), [], 0
    while not rout.IsEnd(idx):
        n = man.IndexToNode(idx)
        path.append(nodes[n])
        nxt = sol.Value(rout.NextVar(idx))
        dist += sub[n][man.IndexToNode(nxt)]
        idx = nxt
    path.append(nodes[man.IndexToNode(idx)])
    return dist, path

# ─── 四、辅助 & 可视化
def format_path(p):
    """压缩重复节点并标注次数；首尾节点不参与计数。"""
    if not p:
        return ""
    start, end = p[0], p[-1]
    internal = p[1:-1]

    # 统计内部节点出现次数，排除 start/end
    counts = Counter(v for v in internal if v not in (start, end))

    parts = [f"{start}(Start)"]
    seen = set()
    for v in internal:
        if v in seen:
            continue
        seen.add(v)
        if v in (start, end):
            parts.append(str(v))
        else:
            c = counts[v]
            parts.append(f"{v}({c})" if c > 1 else str(v))
    parts.append(f"{end}(End)")
    return " -> ".join(parts)

def grid_to_xy(row, col):
    x = MARGIN + (col - 1) * CELL
    y = MARGIN + (ROWS - row) * CELL
    return x, y

class PathWidget(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.full_path   = []
        self.unique_path = []
        self.dup_counts  = {}
        self.specials = {}

    def set_path(self, path, dup_counts=None, specials=None):
        self.full_path = path[:]
        seen, self.unique_path = set(), []
        for v in path:
            if v not in seen:
                seen.add(v)
                self.unique_path.append(v)
        self.dup_counts = dup_counts or {}
        self.specials   = specials or {}
        self._update_canvas()

    def _draw_label(self, x, y, text, dy=12):
        lbl = CoreLabel(text=text, font_size=16, color=(1, 1, 1, 1))
        lbl.refresh()
        w, h = lbl.texture.size
        Rectangle(texture=lbl.texture, pos=(x - w/2, y + dy), size=(w, h))

    def _update_canvas(self):
        self.canvas.clear()
        if not self.unique_path:
            return
        with self.canvas:
            Color(0.35, 0.35, 0.35)
            for (r, c) in PATH_CELLS:
                x, y = grid_to_xy(r, c)
                Rectangle(pos=(x - CELL/2, y - CELL/2), size=(CELL, CELL))

            Color(1, 1, 1)
            for v, (r, c) in NODE_POS.items():
                x, y = grid_to_xy(r, c)
                Ellipse(pos=(x - RADIUS/2, y - RADIUS/2), size=(RADIUS, RADIUS))

            Color(0, 0.6, 0.6)
            for a, b in zip(self.full_path, self.full_path[1:]):
                x1, y1 = grid_to_xy(*NODE_POS[a])
                x2, y2 = grid_to_xy(*NODE_POS[b])
                if x1 == x2 or y1 == y2:
                    Line(points=[x1, y1, x2, y2], width=2)
                else:
                    Line(points=[x1, y1, x2, y1, x2, y2], width=2)

            for v in self.unique_path:
                r, c = NODE_POS[v]
                x, y = grid_to_xy(r, c)

                # 颜色分三种（起点黄，第二点绿，其余红）
                if v == self.specials.get("start"):
                    Color(1, 0.9, 0.1)       # 黄
                elif v == self.specials.get("second"):
                    Color(0.1, 0.7, 0.2)     #绿
                else:
                    Color(0.9, 0.4, 0.3)     # 红

                Ellipse(pos=(x - RADIUS, y - RADIUS), size=(2*RADIUS, 2*RADIUS))

                offset = 20 if v in self.dup_counts else 15
                label  = f"{v}({self.dup_counts[v]})" if v in self.dup_counts else str(v)
                self._draw_label(x + offset, y, label)

# ─── 五、Kivy 应用
class CourierApp(App):
    title = "DhuXhhysrj"

    def build(self):
        root = BoxLayout(orientation="vertical", padding=10, spacing=10)
        self.start_box = self._row(root, "Start (0-31):")
        self.stops_box = self._row(root, "Stops (ps: 6 7 8):")
        self.end_box   = self._row(root, "End   (0-31):")

        btn = Button(text="GoGoGo", size_hint_y=None, height=60)
        btn.bind(on_press=self.compute)
        root.add_widget(btn)

        self.out = Label(halign="left", valign="top", size_hint_y=None, height=160)
        self.out.bind(size=self.out.setter("text_size"))
        root.add_widget(self.out)

        self.out_rev = Label(halign="left", valign="top", size_hint_y=None, height=80)
        self.out_rev.bind(size=self.out_rev.setter("text_size"))
        root.add_widget(self.out_rev)

        self.best_label = Label(halign="left", valign="top", size_hint_y=None, height=80)
        self.best_label.bind(size=self.best_label.setter("text_size"))
        root.add_widget(self.best_label)

        self.path_widget = PathWidget(size_hint_y=1)
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
                raise ValueError("Start / End must be 0-31.")

            stops_raw = self.stops_box.text.strip()
            stops = [int(x) for x in stops_raw.split()] if stops_raw else []
            if any(t < 0 or t > 31 for t in stops):
                raise ValueError("Stops must be 0-31.")
            dup_counts = {k: v for k, v in Counter(stops).items()
                          if v > 1 and k not in (s, e)}

            dist, path = solve_exact(s, stops, e)

            self.out.text = f"Shortest distance: {dist}\n\nPath:\n{format_path(path)}"

            if s == e:
                self.out_rev.text = "Path(reversed):\n" + format_path(path[::-1])
                n = (len(path) + 1) // 2
                first_half  = path[:n]
                second_half = path[n:]

                scount = set(stops)
                def cnt(seq):
                    return sum(1 for v in seq if v in scount)

                best = path if cnt(first_half) > cnt(second_half) else path[::-1]
                self.best_label.text = "Best_Path:\n" + format_path(best)
            else:
                self.out_rev.text   = "Path(reversed):\nNo Cicle"
                self.best_label.text = "Best_Path:\nNo Cicle"
                best = path

            # ---------- 选出黄 & 绿
            start_node  = best[0]
            second_node = next((v for v in best[1:] if v != start_node), None)

            specials = {"start": start_node}
            if second_node is not None:
                specials["second"] = second_node

            # ---------- 可视化
            self.path_widget.set_path(path, dup_counts, specials)

        except Exception as ex:
            self.out.text = f"Error: {ex}"
            self.out_rev.text = ""
            self.best_label.text = ""

# ─── 六、程序入口
if __name__ == "__main__":
    CourierApp().run()
