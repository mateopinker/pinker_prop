import ast
import contextlib
import io
import math
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


PHYSICS_FILE = Path(__file__).with_name("pinker_prop.py")
DEFAULT_INPUT_NAMES = (
    "outer_d",
    "mass",
    "hub_d",
    "blades",
    "sections",
    "motor_rpm",
    "bottom_bracket",
    "top_bracket",
    "exit_step",
    "y",
    "airfoil",
)
STATIC_NAMESPACE_NAMES = ("AIRFOIL_POLARS",)


def load_physics_namespace():
    source = PHYSICS_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(PHYSICS_FILE))

    filtered_body = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom, ast.FunctionDef)):
            filtered_body.append(node)
            continue

        if isinstance(node, ast.Assign):
            target = node.targets[0] if len(node.targets) == 1 else None
            if isinstance(target, ast.Name) and (
                target.id in DEFAULT_INPUT_NAMES or target.id in STATIC_NAMESPACE_NAMES
            ):
                filtered_body.append(node)

    module = ast.Module(body=filtered_body, type_ignores=[])
    ast.fix_missing_locations(module)

    namespace = {}
    exec(compile(module, str(PHYSICS_FILE), "exec"), namespace)
    return namespace


def distribution_to_ui_value(raw_value):
    if raw_value in (1, "1", "ramp"):
        return "ramp"
    return "uniform"


def format_airfoil_label(airfoil):
    return airfoil.upper()


def build_default_inputs(namespace):
    defaults = {name: namespace.get(name) for name in DEFAULT_INPUT_NAMES}
    defaults["y"] = distribution_to_ui_value(defaults.get("y"))
    defaults["airfoil"] = defaults.get("airfoil", "s8025")
    return defaults


def calculate_results(namespace, params):
    solver_log = io.StringIO()
    with contextlib.redirect_stdout(solver_log):
        v_p, f_prop = namespace["inflow"](params["outer_d"], params["mass"])
        f_blade, d_t_list = namespace["thrust_distribution"](
            params["outer_d"],
            params["hub_d"],
            f_prop,
            params["blades"],
            params["sections"],
            params["distribution"],
        )
        v_relative_module, v_relative_angle = namespace["relative_velocity"](
            params["motor_rpm"],
            params["outer_d"],
            params["hub_d"],
            params["sections"],
            v_p,
        )
        chords, aoa, reynolds, thrust, drag, drag_total = namespace["polarpicker"](
            params["top_bracket"],
            params["bottom_bracket"],
            d_t_list,
            v_relative_module,
            v_relative_angle,
            params["exit_step"],
            params["outer_d"],
            params["hub_d"],
            params["sections"],
            airfoil=params["airfoil"],
        )
        geo_pitch = namespace["geometric_pitch_distribution"](v_relative_angle, aoa)

    root_r = params["hub_d"] / 2
    span = (params["outer_d"] / 2) - root_r
    dr = span / params["sections"]
    radius_stations = [root_r + dr * (index + 0.5) for index in range(params["sections"])]

    return {
        "inputs": params,
        "Vp": v_p,
        "F_prop": f_prop,
        "F_blade": f_blade,
        "dT_list": d_t_list,
        "v_relative_module": v_relative_module,
        "v_relative_angle": v_relative_angle,
        "chords": chords,
        "AoA": aoa,
        "Reynolds": reynolds,
        "thrust": thrust,
        "drag": drag,
        "drag_tot": drag_total,
        "geo_pitch": geo_pitch,
        "radius_stations": radius_stations,
        "solver_log": solver_log.getvalue().strip(),
    }


PALETTE = {
    "app_bg": "#edf2f7",
    "sidebar_bg": "#0f172a",
    "sidebar_panel": "#131f38",
    "sidebar_surface": "#17233d",
    "sidebar_edge": "#233354",
    "surface": "#ffffff",
    "surface_alt": "#f6f9fc",
    "surface_edge": "#d8e1eb",
    "ink": "#0f172a",
    "muted": "#64748b",
    "muted_light": "#94a3b8",
    "accent": "#0ea5e9",
    "accent_dark": "#075985",
    "accent_bright": "#38bdf8",
    "accent_soft": "#e0f2fe",
    "accent_soft_text": "#0c4a6e",
    "success_bg": "#dcfce7",
    "success_fg": "#166534",
    "warning_bg": "#fef3c7",
    "warning_fg": "#92400e",
    "error_bg": "#fee2e2",
    "error_fg": "#991b1b",
    "console_bg": "#0b1120",
    "console_fg": "#e2e8f0",
    "field_bg": "#f8fbff",
    "field_focus_bg": "#ecfeff",
    "field_border": "#cbd5e1",
    "field_value": "#0f4c81",
}

FONT_FAMILY = "DejaVu Sans"
MONO_FAMILY = "DejaVu Sans Mono"

FIELD_GROUPS = (
    (
        "Geometry",
        "Set the airframe footprint that drives the hover case.",
        (
            ("outer_d", "Outer diameter", "m"),
            ("hub_d", "Hub diameter", "m"),
            ("mass", "Aircraft mass", "kg"),
        ),
    ),
    (
        "Rotor",
        "Tune the rotating hardware and radial resolution.",
        (
            ("blades", "Blade count", ""),
            ("sections", "Sections", ""),
            ("motor_rpm", "Motor speed", "rpm"),
        ),
    ),
    (
        "Solver",
        "Bracket the chord search without touching the model.",
        (
            ("bottom_bracket", "Min chord", "m"),
            ("top_bracket", "Max chord", "m"),
            ("exit_step", "Exit step", ""),
        ),
    ),
)

METRIC_SPECS = (
    ("Vp", "Inflow", "m/s", "#e0f2fe", "#0c4a6e", "#0284c7"),
    ("F_prop", "Motor thrust", "N", "#ecfdf5", "#166534", "#15803d"),
    ("F_blade", "Blade thrust", "N", "#fff7ed", "#9a3412", "#c2410c"),
    ("drag_tot", "Total drag", "N", "#fef3c7", "#92400e", "#b45309"),
)


class PinkerPropUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pinker Prop Studio")
        self.root.geometry("1380x860")
        self.root.minsize(1220, 760)
        self.is_fullscreen = False

        self.namespace = load_physics_namespace()
        self.defaults = build_default_inputs(self.namespace)
        self.available_airfoils = self.namespace["list_available_airfoils"]()
        self.input_vars = {}
        self.input_field_refs = {}
        self.summary_vars = {}
        self.meta_vars = {
            "distribution": tk.StringVar(value="Distribution: -"),
            "airfoil": tk.StringVar(value="Airfoil: -"),
            "layout": tk.StringVar(value="Blades: - | Sections: -"),
        }
        self.status_detail_var = tk.StringVar(value="Ready.")
        self.distribution_buttons = {}
        self.airfoil_buttons = {}

        self._configure_styles()
        self._build_layout()
        self._reset_to_defaults()
        self._configure_window_mode()

    def _configure_styles(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")

        self.root.configure(bg=PALETTE["app_bg"])

        style.configure("Dashboard.TNotebook", background=PALETTE["app_bg"], borderwidth=0)
        style.configure(
            "Dashboard.TNotebook.Tab",
            background=PALETTE["surface_alt"],
            foreground=PALETTE["muted"],
            padding=(18, 10),
            font=(FONT_FAMILY, 10, "bold"),
            borderwidth=0,
        )
        style.map(
            "Dashboard.TNotebook.Tab",
            background=[("selected", PALETTE["surface"])],
            foreground=[("selected", PALETTE["ink"])],
        )

        style.configure(
            "Results.Treeview",
            background=PALETTE["surface"],
            fieldbackground=PALETTE["surface"],
            foreground=PALETTE["ink"],
            borderwidth=0,
            rowheight=34,
            font=(MONO_FAMILY, 10),
        )
        style.map(
            "Results.Treeview",
            background=[("selected", PALETTE["accent_soft"])],
            foreground=[("selected", PALETTE["ink"])],
        )
        style.configure(
            "Results.Treeview.Heading",
            background=PALETTE["ink"],
            foreground=PALETTE["surface"],
            borderwidth=0,
            relief="flat",
            padding=(12, 10),
            font=(FONT_FAMILY, 10, "bold"),
        )
        style.map("Results.Treeview.Heading", background=[("active", PALETTE["ink"])])

    def _build_layout(self):
        shell = tk.Frame(self.root, bg=PALETTE["app_bg"], padx=24, pady=24)
        shell.pack(fill="both", expand=True)
        shell.grid_columnconfigure(1, weight=1)
        shell.grid_rowconfigure(0, weight=1)

        sidebar = tk.Frame(
            shell,
            bg=PALETTE["sidebar_bg"],
            width=940,
            padx=28,
            pady=24,
            highlightthickness=1,
            highlightbackground=PALETTE["sidebar_edge"],
        )
        sidebar.grid(row=0, column=0, sticky="nsew")
        sidebar.grid_propagate(False)
        sidebar.grid_columnconfigure(0, weight=1)
        sidebar.grid_rowconfigure(6, weight=1)

        content = tk.Frame(shell, bg=PALETTE["app_bg"])
        content.grid(row=0, column=1, sticky="nsew", padx=(24, 0))
        content.grid_columnconfigure(0, weight=1)
        content.grid_rowconfigure(2, weight=1)

        self._build_sidebar(sidebar)
        self._build_content(content)

    def _build_sidebar(self, parent):
        intro = tk.Frame(parent, bg=PALETTE["sidebar_bg"])
        intro.grid(row=0, column=0, sticky="ew")

        tk.Label(
            intro,
            text="PINKER PROP",
            font=(FONT_FAMILY, 10, "bold"),
            fg="#7dd3fc",
            bg=PALETTE["sidebar_bg"],
        ).pack(anchor="w")
        tk.Label(
            intro,
            text="Shape the hover case",
            font=(FONT_FAMILY, 22, "bold"),
            fg=PALETTE["surface"],
            bg=PALETTE["sidebar_bg"],
        ).pack(anchor="w", pady=(8, 6))
        tk.Label(
            intro,
            text="A cleaner front-end for your propeller model. The physics file stays untouched.",
            font=(FONT_FAMILY, 10),
            fg=PALETTE["muted_light"],
            bg=PALETTE["sidebar_bg"],
            justify="left",
            wraplength=360,
        ).pack(anchor="w")
        tk.Label(
            intro,
            text=f"Locked to {PHYSICS_FILE.name}",
            font=(FONT_FAMILY, 9, "bold"),
            fg="#7dd3fc",
            bg=PALETTE["sidebar_panel"],
            padx=10,
            pady=6,
        ).pack(anchor="w", pady=(14, 0))

        for row_index, (title, description, fields) in enumerate(FIELD_GROUPS, start=1):
            card = self._create_sidebar_card(parent)
            card.grid(row=row_index, column=0, sticky="ew", pady=(18 if row_index == 1 else 12, 0))
            self._build_field_group(card, title, description, fields)

        airfoil_card = self._create_sidebar_card(parent)
        airfoil_card.grid(row=4, column=0, sticky="ew", pady=(12, 0))
        self._build_airfoil_group(airfoil_card)

        profile_card = self._create_sidebar_card(parent)
        profile_card.grid(row=5, column=0, sticky="ew", pady=(12, 0))
        self._build_distribution_group(profile_card)

        footer = self._create_sidebar_card(parent)
        footer.grid(row=7, column=0, sticky="sew", pady=(12, 0))
        tk.Label(
            footer,
            text="Status",
            font=(FONT_FAMILY, 10, "bold"),
            fg=PALETTE["surface"],
            bg=PALETTE["sidebar_panel"],
        ).pack(anchor="w")
        self.sidebar_status_label = tk.Label(
            footer,
            textvariable=self.status_detail_var,
            font=(FONT_FAMILY, 10),
            fg=PALETTE["muted_light"],
            bg=PALETTE["sidebar_panel"],
            justify="left",
            wraplength=360,
        )
        self.sidebar_status_label.pack(anchor="w", pady=(8, 16))

        button_row = tk.Frame(footer, bg=PALETTE["sidebar_panel"])
        button_row.pack(fill="x")
        self._make_button(
            button_row,
            text="Reset defaults",
            command=self._reset_to_defaults,
            bg=PALETTE["sidebar_surface"],
            fg=PALETTE["surface"],
        ).pack(fill="x")

    def _build_content(self, parent):
        hero = tk.Frame(
            parent,
            bg=PALETTE["surface"],
            padx=28,
            pady=24,
            highlightthickness=1,
            highlightbackground=PALETTE["surface_edge"],
        )
        hero.grid(row=0, column=0, sticky="ew")
        hero.grid_columnconfigure(1, weight=1)

        accent_strip = tk.Frame(hero, bg=PALETTE["accent"], width=8)
        accent_strip.grid(row=0, column=0, sticky="ns", padx=(0, 20))

        hero_text = tk.Frame(hero, bg=PALETTE["surface"])
        hero_text.grid(row=0, column=1, sticky="w")
        tk.Label(
            hero_text,
            text="Propeller Concept Studio",
            font=(FONT_FAMILY, 24, "bold"),
            fg=PALETTE["ink"],
            bg=PALETTE["surface"],
        ).pack(anchor="w")
        tk.Label(
            hero_text,
            text="Fast hover sizing, cleaner presentation, same underlying solver.",
            font=(FONT_FAMILY, 11),
            fg=PALETTE["muted"],
            bg=PALETTE["surface"],
        ).pack(anchor="w", pady=(6, 14))

        meta_row = tk.Frame(hero_text, bg=PALETTE["surface"])
        meta_row.pack(anchor="w")
        self._make_pill(
            meta_row,
            text=f"Source: {PHYSICS_FILE.name}",
            bg=PALETTE["accent_soft"],
            fg=PALETTE["accent_soft_text"],
        ).pack(side="left")
        self.meta_distribution_label = self._make_pill(
            meta_row,
            textvariable=self.meta_vars["distribution"],
            bg=PALETTE["surface_alt"],
            fg=PALETTE["ink"],
        )
        self.meta_distribution_label.pack(side="left", padx=(10, 0))
        self.meta_airfoil_label = self._make_pill(
            meta_row,
            textvariable=self.meta_vars["airfoil"],
            bg=PALETTE["surface_alt"],
            fg=PALETTE["ink"],
        )
        self.meta_airfoil_label.pack(side="left", padx=(10, 0))
        self.meta_layout_label = self._make_pill(
            meta_row,
            textvariable=self.meta_vars["layout"],
            bg=PALETTE["surface_alt"],
            fg=PALETTE["ink"],
        )
        self.meta_layout_label.pack(side="left", padx=(10, 0))

        actions = tk.Frame(hero, bg=PALETTE["surface"])
        actions.grid(row=0, column=2, sticky="e")
        self.status_badge = self._make_pill(
            actions,
            text="Ready",
            bg=PALETTE["warning_bg"],
            fg=PALETTE["warning_fg"],
        )
        self.status_badge.pack(anchor="e")
        self._make_button(
            actions,
            text="Run analysis",
            command=self.run_analysis,
            bg=PALETTE["accent"],
            fg=PALETTE["surface"],
            active_bg=PALETTE["accent_dark"],
        ).pack(anchor="e", pady=(14, 10))
        self._make_button(
            actions,
            text="Reset defaults",
            command=self._reset_to_defaults,
            bg=PALETTE["surface_alt"],
            fg=PALETTE["ink"],
            active_bg="#e7edf4",
        ).pack(anchor="e")

        metrics = tk.Frame(parent, bg=PALETTE["app_bg"])
        metrics.grid(row=1, column=0, sticky="ew", pady=(18, 0))
        for column in range(len(METRIC_SPECS)):
            metrics.grid_columnconfigure(column, weight=1)

        for column, (key, title, unit, badge_bg, badge_fg, value_fg) in enumerate(METRIC_SPECS):
            card = tk.Frame(
                metrics,
                bg=PALETTE["surface"],
                padx=20,
                pady=18,
                highlightthickness=1,
                highlightbackground=PALETTE["surface_edge"],
            )
            card.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 12, 0))
            top_row = tk.Frame(card, bg=PALETTE["surface"])
            top_row.pack(fill="x")
            tk.Label(
                top_row,
                text=title,
                font=(FONT_FAMILY, 10, "bold"),
                fg=PALETTE["ink"],
                bg=PALETTE["surface"],
            ).pack(side="left")
            tk.Label(
                top_row,
                text=unit,
                font=(FONT_FAMILY, 9, "bold"),
                fg=badge_fg,
                bg=badge_bg,
                padx=8,
                pady=4,
            ).pack(side="right")
            value_var = tk.StringVar(value="--")
            tk.Label(
                card,
                textvariable=value_var,
                font=(FONT_FAMILY, 22, "bold"),
                fg=value_fg,
                bg=PALETTE["surface"],
            ).pack(anchor="w", pady=(16, 4))
            tk.Label(
                card,
                text="Live from the current run",
                font=(FONT_FAMILY, 9),
                fg=PALETTE["muted"],
                bg=PALETTE["surface"],
            ).pack(anchor="w")
            self.summary_vars[key] = value_var

        workspace = tk.Frame(parent, bg=PALETTE["app_bg"])
        workspace.grid(row=2, column=0, sticky="nsew", pady=(18, 0))
        workspace.grid_columnconfigure(0, weight=1)
        workspace.grid_rowconfigure(0, weight=1)

        self.notebook = ttk.Notebook(workspace, style="Dashboard.TNotebook")
        self.notebook.grid(row=0, column=0, sticky="nsew")

        sections_tab = tk.Frame(self.notebook, bg=PALETTE["surface"])
        solver_tab = tk.Frame(self.notebook, bg=PALETTE["surface"])
        self.notebook.add(sections_tab, text="Blade Sections")
        self.notebook.add(solver_tab, text="Solver Trace")

        self._build_sections_tab(sections_tab)
        self._build_solver_tab(solver_tab)

    def _build_sections_tab(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(1, weight=1)

        header = tk.Frame(parent, bg=PALETTE["surface"], padx=20, pady=18)
        header.grid(row=0, column=0, sticky="ew")
        tk.Label(
            header,
            text="Blade section map",
            font=(FONT_FAMILY, 15, "bold"),
            fg=PALETTE["ink"],
            bg=PALETTE["surface"],
        ).pack(anchor="w")
        tk.Label(
            header,
            text="Scrollable radial breakdown with cleaner numeric formatting.",
            font=(FONT_FAMILY, 10),
            fg=PALETTE["muted"],
            bg=PALETTE["surface"],
        ).pack(anchor="w", pady=(4, 0))

        table_shell = tk.Frame(
            parent,
            bg=PALETTE["surface"],
            padx=16,
            pady=0,
            highlightthickness=1,
            highlightbackground=PALETTE["surface_edge"],
        )
        table_shell.grid(row=1, column=0, sticky="nsew", padx=20, pady=(0, 20))
        table_shell.grid_columnconfigure(0, weight=1)
        table_shell.grid_rowconfigure(0, weight=1)

        columns = (
            "section",
            "radius",
            "dT",
            "vrel",
            "phi",
            "chord",
            "aoa",
            "re",
            "drag",
            "pitch",
        )
        self.table = ttk.Treeview(
            table_shell,
            columns=columns,
            show="headings",
            style="Results.Treeview",
            selectmode="browse",
        )

        headings = {
            "section": "Sec",
            "radius": "Radius",
            "dT": "dT",
            "vrel": "V rel",
            "phi": "Phi",
            "chord": "Chord",
            "aoa": "AoA",
            "re": "Re",
            "drag": "Drag",
            "pitch": "Pitch",
        }
        widths = {
            "section": 64,
            "radius": 116,
            "dT": 110,
            "vrel": 118,
            "phi": 98,
            "chord": 116,
            "aoa": 98,
            "re": 136,
            "drag": 110,
            "pitch": 110,
        }
        anchors = {
            "section": "center",
            "radius": "e",
            "dT": "e",
            "vrel": "e",
            "phi": "e",
            "chord": "e",
            "aoa": "e",
            "re": "e",
            "drag": "e",
            "pitch": "e",
        }

        for column in columns:
            self.table.heading(column, text=headings[column])
            self.table.column(
                column,
                width=widths[column],
                minwidth=widths[column],
                anchor=anchors[column],
                stretch=True,
            )

        self.table.tag_configure("odd", background=PALETTE["surface"])
        self.table.tag_configure("even", background=PALETTE["surface_alt"])

        y_scroll = ttk.Scrollbar(table_shell, orient="vertical", command=self.table.yview)
        x_scroll = ttk.Scrollbar(table_shell, orient="horizontal", command=self.table.xview)
        self.table.configure(yscrollcommand=y_scroll.set, xscrollcommand=x_scroll.set)
        self.table.grid(row=0, column=0, sticky="nsew")
        y_scroll.grid(row=0, column=1, sticky="ns")
        x_scroll.grid(row=1, column=0, sticky="ew")

    def _build_solver_tab(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_rowconfigure(1, weight=1)

        header = tk.Frame(parent, bg=PALETTE["surface"], padx=20, pady=18)
        header.grid(row=0, column=0, sticky="ew")
        tk.Label(
            header,
            text="Solver trace",
            font=(FONT_FAMILY, 15, "bold"),
            fg=PALETTE["ink"],
            bg=PALETTE["surface"],
        ).pack(anchor="w")
        tk.Label(
            header,
            text="Direct stdout captured from the unchanged physics workflow.",
            font=(FONT_FAMILY, 10),
            fg=PALETTE["muted"],
            bg=PALETTE["surface"],
        ).pack(anchor="w", pady=(4, 0))

        log_shell = tk.Frame(
            parent,
            bg=PALETTE["console_bg"],
            highlightthickness=1,
            highlightbackground="#162033",
        )
        log_shell.grid(row=1, column=0, sticky="nsew", padx=20, pady=(0, 20))
        log_shell.grid_columnconfigure(0, weight=1)
        log_shell.grid_rowconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(
            log_shell,
            wrap="word",
            state="disabled",
            bg=PALETTE["console_bg"],
            fg=PALETTE["console_fg"],
            insertbackground="#7dd3fc",
            relief="flat",
            borderwidth=0,
            font=(MONO_FAMILY, 10),
            padx=18,
            pady=18,
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")

    def _create_sidebar_card(self, parent):
        return tk.Frame(
            parent,
            bg=PALETTE["sidebar_panel"],
            padx=16,
            pady=16,
            highlightthickness=1,
            highlightbackground=PALETTE["sidebar_edge"],
        )

    def _build_field_group(self, parent, title, description, fields):
        tk.Label(
            parent,
            text=title,
            font=(FONT_FAMILY, 11, "bold"),
            fg=PALETTE["surface"],
            bg=PALETTE["sidebar_panel"],
        ).pack(anchor="w")
        tk.Label(
            parent,
            text=description,
            font=(FONT_FAMILY, 9),
            fg=PALETTE["muted_light"],
            bg=PALETTE["sidebar_panel"],
            justify="left",
            wraplength=360,
        ).pack(anchor="w", pady=(4, 12))

        grid = tk.Frame(parent, bg=PALETTE["sidebar_panel"])
        grid.pack(fill="x")
        grid.grid_columnconfigure(0, weight=1)
        grid.grid_columnconfigure(1, weight=1)

        for index, (key, label, unit) in enumerate(fields):
            row = index // 2
            column = index % 2
            span = 2 if len(fields) % 2 == 1 and index == len(fields) - 1 else 1
            self._build_input_field(grid, row, column, span, key, label, unit)

    def _build_input_field(self, parent, row, column, columnspan, key, label, unit):
        field = tk.Frame(
            parent,
            bg=PALETTE["sidebar_surface"],
            padx=12,
            pady=10,
            highlightthickness=1,
            highlightbackground=PALETTE["sidebar_edge"],
        )
        padx = (0, 6) if column == 0 and columnspan == 1 else (6, 0)
        if columnspan == 2:
            padx = (0, 0)
        field.grid(row=row, column=column, columnspan=columnspan, sticky="ew", padx=padx, pady=(0, 8))

        top = tk.Frame(field, bg=PALETTE["sidebar_surface"])
        top.pack(fill="x")
        top_label = tk.Label(
            top,
            text=label.upper(),
            font=(FONT_FAMILY, 8, "bold"),
            fg=PALETTE["muted_light"],
            bg=PALETTE["sidebar_surface"],
        )
        top_label.pack(side="left")
        unit_label = None
        if unit:
            unit_label = tk.Label(
                top,
                text=unit,
                font=(FONT_FAMILY, 8, "bold"),
                fg="#7dd3fc",
                bg=PALETTE["sidebar_surface"],
            )
            unit_label.pack(side="right")

        input_shell = tk.Frame(
            field,
            bg=PALETTE["field_bg"],
            padx=10,
            pady=8,
            highlightthickness=1,
            highlightbackground=PALETTE["field_border"],
        )
        input_shell.pack(fill="x", pady=(10, 0))
        var = tk.StringVar()
        entry = tk.Entry(
            input_shell,
            textvariable=var,
            relief="flat",
            borderwidth=0,
            highlightthickness=0,
            bg=PALETTE["field_bg"],
            fg=PALETTE["field_value"],
            insertbackground=PALETTE["accent_dark"],
            selectbackground=PALETTE["accent"],
            selectforeground=PALETTE["surface"],
            font=(FONT_FAMILY, 12, "bold"),
        )
        entry.pack(fill="x")
        self.input_vars[key] = var
        self.input_field_refs[key] = {
            "field": field,
            "input_shell": input_shell,
            "label": top_label,
            "unit": unit_label,
            "entry": entry,
        }
        entry.bind("<FocusIn>", lambda _event, name=key: self._set_input_highlight(name, True))
        entry.bind("<FocusOut>", lambda _event, name=key: self._set_input_highlight(name, False))
        self._set_input_highlight(key, False)

    def _build_distribution_group(self, parent):
        tk.Label(
            parent,
            text="Distribution",
            font=(FONT_FAMILY, 11, "bold"),
            fg=PALETTE["surface"],
            bg=PALETTE["sidebar_panel"],
        ).pack(anchor="w")
        tk.Label(
            parent,
            text="Switch between a flat span loading and a tip-biased ramp.",
            font=(FONT_FAMILY, 9),
            fg=PALETTE["muted_light"],
            bg=PALETTE["sidebar_panel"],
            justify="left",
            wraplength=360,
        ).pack(anchor="w", pady=(4, 12))

        self.distribution_var = tk.StringVar(value="uniform")
        toggle = tk.Frame(parent, bg=PALETTE["sidebar_surface"], padx=4, pady=4)
        toggle.pack(fill="x")
        toggle.grid_columnconfigure(0, weight=1)
        toggle.grid_columnconfigure(1, weight=1)

        for column, mode in enumerate(("uniform", "ramp")):
            button = tk.Button(
                toggle,
                text=mode.title(),
                command=lambda selected=mode: self._set_distribution(selected),
                relief="flat",
                borderwidth=0,
                highlightthickness=0,
                cursor="hand2",
                font=(FONT_FAMILY, 10, "bold"),
                padx=14,
                pady=10,
            )
            button.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 6, 0))
            self.distribution_buttons[mode] = button

    def _build_airfoil_group(self, parent):
        tk.Label(
            parent,
            text="Airfoil",
            font=(FONT_FAMILY, 11, "bold"),
            fg=PALETTE["surface"],
            bg=PALETTE["sidebar_panel"],
        ).pack(anchor="w")
        tk.Label(
            parent,
            text="Choose one airfoil dataset for the whole blade span.",
            font=(FONT_FAMILY, 9),
            fg=PALETTE["muted_light"],
            bg=PALETTE["sidebar_panel"],
            justify="left",
            wraplength=360,
        ).pack(anchor="w", pady=(4, 12))

        toggle = tk.Frame(parent, bg=PALETTE["sidebar_surface"], padx=4, pady=4)
        toggle.pack(fill="x")

        columns = 2
        for column in range(columns):
            toggle.grid_columnconfigure(column, weight=1)

        self.airfoil_var = tk.StringVar(value=self.defaults["airfoil"])
        for index, airfoil in enumerate(self.available_airfoils):
            row = index // columns
            column = index % columns
            button = tk.Button(
                toggle,
                text=format_airfoil_label(airfoil),
                command=lambda selected=airfoil: self._set_airfoil(selected),
                relief="flat",
                borderwidth=0,
                highlightthickness=0,
                cursor="hand2",
                font=(FONT_FAMILY, 10, "bold"),
                padx=14,
                pady=10,
            )
            button.grid(
                row=row,
                column=column,
                sticky="ew",
                padx=(0 if column == 0 else 6, 0),
                pady=(0 if row == 0 else 6, 0),
            )
            self.airfoil_buttons[airfoil] = button

    def _make_pill(self, parent, text=None, textvariable=None, bg=None, fg=None):
        label = tk.Label(
            parent,
            text=text,
            textvariable=textvariable,
            bg=bg or PALETTE["surface_alt"],
            fg=fg or PALETTE["ink"],
            font=(FONT_FAMILY, 9, "bold"),
            padx=12,
            pady=6,
        )
        return label

    def _make_button(self, parent, text, command, bg, fg, active_bg=None):
        return tk.Button(
            parent,
            text=text,
            command=command,
            relief="flat",
            borderwidth=0,
            highlightthickness=0,
            cursor="hand2",
            bg=bg,
            fg=fg,
            activebackground=active_bg or bg,
            activeforeground=fg,
            font=(FONT_FAMILY, 10, "bold"),
            padx=18,
            pady=12,
        )

    def _configure_window_mode(self):
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.bind("<Escape>", self._exit_fullscreen)
        self.root.after(0, self._enter_fullscreen)

    def _enter_fullscreen(self, event=None):
        self.is_fullscreen = True
        try:
            self.root.attributes("-fullscreen", True)
        except tk.TclError:
            try:
                self.root.attributes("-zoomed", True)
            except tk.TclError:
                try:
                    self.root.state("zoomed")
                except tk.TclError:
                    pass
        return "break" if event is not None else None

    def _exit_fullscreen(self, event=None):
        self.is_fullscreen = False
        try:
            self.root.attributes("-fullscreen", False)
        except tk.TclError:
            pass
        try:
            self.root.attributes("-zoomed", False)
        except tk.TclError:
            pass
        try:
            self.root.state("normal")
        except tk.TclError:
            pass
        return "break" if event is not None else None

    def _toggle_fullscreen(self, event=None):
        if self.is_fullscreen:
            return self._exit_fullscreen(event)
        return self._enter_fullscreen(event)

    def _set_input_highlight(self, key, active):
        refs = self.input_field_refs.get(key)
        if not refs:
            return

        field_border = PALETTE["accent_bright"] if active else PALETTE["sidebar_edge"]
        shell_bg = PALETTE["field_focus_bg"] if active else PALETTE["field_bg"]
        shell_border = PALETTE["accent"] if active else PALETTE["field_border"]
        label_fg = "#7dd3fc" if active else PALETTE["muted_light"]
        value_fg = PALETTE["accent_dark"] if active else PALETTE["field_value"]

        refs["field"].configure(highlightbackground=field_border)
        refs["input_shell"].configure(bg=shell_bg, highlightbackground=shell_border)
        refs["entry"].configure(bg=shell_bg, fg=value_fg, insertbackground=value_fg)
        refs["label"].configure(fg=label_fg)
        if refs["unit"] is not None:
            refs["unit"].configure(fg=label_fg)

    def _set_airfoil(self, airfoil):
        self.airfoil_var.set(airfoil)
        for name, button in self.airfoil_buttons.items():
            if name == airfoil:
                button.configure(
                    bg=PALETTE["accent"],
                    fg=PALETTE["surface"],
                    activebackground=PALETTE["accent_dark"],
                    activeforeground=PALETTE["surface"],
                )
            else:
                button.configure(
                    bg=PALETTE["sidebar_surface"],
                    fg=PALETTE["muted_light"],
                    activebackground=PALETTE["sidebar_surface"],
                    activeforeground=PALETTE["surface"],
                )

    def _set_distribution(self, mode):
        self.distribution_var.set(mode)
        for name, button in self.distribution_buttons.items():
            if name == mode:
                button.configure(
                    bg=PALETTE["accent"],
                    fg=PALETTE["surface"],
                    activebackground=PALETTE["accent_dark"],
                    activeforeground=PALETTE["surface"],
                )
            else:
                button.configure(
                    bg=PALETTE["sidebar_surface"],
                    fg=PALETTE["muted_light"],
                    activebackground=PALETTE["sidebar_surface"],
                    activeforeground=PALETTE["surface"],
                )

    def _set_status(self, badge_text, detail_text, tone):
        tones = {
            "idle": (PALETTE["warning_bg"], PALETTE["warning_fg"]),
            "running": (PALETTE["accent_soft"], PALETTE["accent_soft_text"]),
            "success": (PALETTE["success_bg"], PALETTE["success_fg"]),
            "error": (PALETTE["error_bg"], PALETTE["error_fg"]),
        }
        bg, fg = tones[tone]
        self.status_badge.configure(text=badge_text, bg=bg, fg=fg)
        self.status_detail_var.set(detail_text)
        detail_color = {
            "idle": PALETTE["muted_light"],
            "running": "#7dd3fc",
            "success": "#86efac",
            "error": "#fda4af",
        }
        self.sidebar_status_label.configure(fg=detail_color[tone])

    def _refresh_meta(self, params=None):
        if params is None:
            self.meta_vars["distribution"].set(f"Distribution: {self.distribution_var.get().title()}")
            self.meta_vars["airfoil"].set(
                f"Airfoil: {format_airfoil_label(self.airfoil_var.get())}"
            )
            blades = self.input_vars["blades"].get() or "-"
            sections = self.input_vars["sections"].get() or "-"
        else:
            self.meta_vars["distribution"].set(f"Distribution: {params['distribution'].title()}")
            self.meta_vars["airfoil"].set(
                f"Airfoil: {format_airfoil_label(params['airfoil'])}"
            )
            blades = params["blades"]
            sections = params["sections"]
        self.meta_vars["layout"].set(f"Blades: {blades} | Sections: {sections}")

    def _reset_to_defaults(self):
        for key, value in self.defaults.items():
            if key == "y":
                self._set_distribution(value)
                continue
            if key == "airfoil":
                self._set_airfoil(value)
                continue
            self.input_vars[key].set(str(value))
        self._refresh_meta()
        self._set_status("Defaults loaded", "Inputs synced with pinker_prop.py.", "idle")

    def _read_inputs(self):
        try:
            params = {
                "outer_d": float(self.input_vars["outer_d"].get()),
                "mass": float(self.input_vars["mass"].get()),
                "hub_d": float(self.input_vars["hub_d"].get()),
                "blades": int(self.input_vars["blades"].get()),
                "sections": int(self.input_vars["sections"].get()),
                "motor_rpm": float(self.input_vars["motor_rpm"].get()),
                "bottom_bracket": float(self.input_vars["bottom_bracket"].get()),
                "top_bracket": float(self.input_vars["top_bracket"].get()),
                "exit_step": float(self.input_vars["exit_step"].get()),
                "distribution": self.distribution_var.get(),
                "airfoil": self.airfoil_var.get(),
            }
        except ValueError as exc:
            raise ValueError("All inputs must contain valid numbers.") from exc

        if params["outer_d"] <= 0 or params["mass"] <= 0 or params["hub_d"] <= 0:
            raise ValueError("Diameters and mass must be greater than zero.")
        if params["hub_d"] >= params["outer_d"]:
            raise ValueError("Hub diameter must be smaller than outer diameter.")
        if params["blades"] <= 0 or params["sections"] <= 0:
            raise ValueError("Blade count and section count must be positive integers.")
        if params["bottom_bracket"] <= 0 or params["top_bracket"] <= 0:
            raise ValueError("Chord brackets must be greater than zero.")
        if params["bottom_bracket"] >= params["top_bracket"]:
            raise ValueError("Min chord must be smaller than max chord.")
        if params["exit_step"] <= 0:
            raise ValueError("Exit step must be greater than zero.")
        if params["distribution"] not in {"uniform", "ramp"}:
            raise ValueError("Distribution must be uniform or ramp.")
        if params["airfoil"] not in self.available_airfoils:
            raise ValueError(
                "Airfoil must be one of: "
                + ", ".join(format_airfoil_label(name) for name in self.available_airfoils)
            )

        return params

    def run_analysis(self):
        self._set_status("Running", "Pushing the current inputs through the existing solver.", "running")
        self.root.update_idletasks()
        try:
            params = self._read_inputs()
            results = calculate_results(self.namespace, params)
        except Exception as exc:
            self._set_status("Needs attention", str(exc), "error")
            messagebox.showerror("Analysis Error", str(exc))
            return

        self.summary_vars["Vp"].set(f"{results['Vp']:.4f}")
        self.summary_vars["F_prop"].set(f"{results['F_prop']:.4f}")
        self.summary_vars["F_blade"].set(f"{results['F_blade']:.4f}")
        self.summary_vars["drag_tot"].set(f"{results['drag_tot']:.5f}")
        self._refresh_meta(params)

        self.table.delete(*self.table.get_children())
        for index in range(params["sections"]):
            self.table.insert(
                "",
                "end",
                values=(
                    index + 1,
                    f"{results['radius_stations'][index]:.4f}",
                    f"{results['thrust'][index]:.5f}",
                    f"{results['v_relative_module'][index]:.3f}",
                    f"{math.degrees(results['v_relative_angle'][index]):.2f}",
                    f"{results['chords'][index]:.5f}",
                    f"{results['AoA'][index]:.2f}",
                    f"{results['Reynolds'][index]:.0f}",
                    f"{results['drag'][index]:.5f}",
                    f"{math.degrees(results['geo_pitch'][index]):.2f}",
                ),
                tags=("even" if index % 2 == 0 else "odd",),
            )

        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert("1.0", results["solver_log"] or "No solver log output.")
        self.log_text.configure(state="disabled")

        self._set_status("Live result", "Analysis complete. Metrics and blade sections are up to date.", "success")


def main():
    root = tk.Tk()
    app = PinkerPropUI(root)
    app.run_analysis()
    root.mainloop()


if __name__ == "__main__":
    main()
