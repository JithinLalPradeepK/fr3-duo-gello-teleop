"""Franka Robotics brand theme for the demo GUI (Brand Guidelines, February 2027 - docs/brand/).

Brand rules applied:
- Colours: Franka Black #000000 (text, logo), Franka White #FFFFFF, Dark Blue #1B2F36 (backgrounds/accents),
  Franka Grey #587380 (structure, buttons), Off-White #D4DBDC (text on dark, lines). Accent colours only small
  (lights, switch tracks), never as large backgrounds; Franka Mint = "OK / on".
- Fonts: Montserrat (headlines in CAPITALS, Medium; highlights SemiBold) from assets/brand/fonts (SIL OFL),
  Lato (supporting text), RM Mono for technical data -> not available, DejaVu Sans Mono stands in.
- Logo: black/white only, never below 175 px wide, top-left, >= 48 px from the side, >= 40 px from the top.
FUNCTIONAL SAFETY COLOURS ARE KEPT ON PURPOSE (not brand colours): START green, STOP / ERROR red,
motion-in-progress amber. The GUI is not a safety device; the physical E-stop stays in use.
"""
import os

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFontDatabase
from PyQt5.QtSvg import QSvgWidget

DEMO = os.path.expanduser('~/robot_demo')
ASSETS = os.path.join(DEMO, 'assets', 'brand')
FONT_DIR = os.path.join(ASSETS, 'fonts')
LOGO = os.path.join(ASSETS, 'FrankaRobotics_CompanyLogo_Digital_RGB.svg')
LOGO_NEG = os.path.join(ASSETS, 'FrankaRobotics_CompanyLogo_negative_Digital_RGB.svg')

# brand palette
BLACK, WHITE = '#000000', '#FFFFFF'
DARK_BLUE, GREY, OFF_WHITE = '#1B2F36', '#587380', '#D4DBDC'
MINT, SKY, COBALT, IRIS = '#75C28B', '#88D4FF', '#567FC5', '#7884E2'
PANEL = '#F3F5F5'                       # Off-White lightened towards white (subtle card background)
# functional safety colours (deliberately outside the brand palette)
SAFE_GREEN, SAFE_RED, SAFE_AMBER = '#2E7D32', '#C62828', '#E69500'

# status chips: (background, text)
LIGHT_OK, LIGHT_WARN, LIGHT_BAD, LIGHT_OFF = (MINT, BLACK), (SAFE_AMBER, BLACK), (SAFE_RED, WHITE), (OFF_WHITE, GREY)

STATE_COLORS = {'IDLE': DARK_BLUE, 'TELEOP': SAFE_GREEN, 'TELEOP_STARTING': SAFE_AMBER, 'HOMING': SAFE_AMBER,
                'STOPPING': SAFE_AMBER, 'STARTING_UP': SAFE_AMBER, 'RECOVERING': SAFE_AMBER,
                'SHUTTING_DOWN': SAFE_AMBER, 'ERROR': SAFE_RED, 'OFFLINE': GREY}

# button roles: (background, text)
BUTTON = {'start': (SAFE_GREEN, WHITE), 'stop': (SAFE_RED, WHITE), 'home': (DARK_BLUE, WHITE),
          'primary': (DARK_BLUE, WHITE), 'secondary': (GREY, WHITE)}

HEAD, TEXT, MONO = 'Montserrat', 'Lato', 'RM Mono'


def load_fonts():
    """Register brand fonts from assets/brand/fonts (if present); return the families in use."""
    global HEAD, MONO
    if os.path.isdir(FONT_DIR):
        for f in sorted(os.listdir(FONT_DIR)):
            if f.lower().endswith(('.ttf', '.otf')):
                QFontDatabase.addApplicationFont(os.path.join(FONT_DIR, f))
    fams = set(QFontDatabase().families())
    HEAD = 'Montserrat' if 'Montserrat' in fams else 'Lato'
    MONO = next((f for f in ('RM Mono', 'DejaVu Sans Mono') if f in fams), 'Monospace')
    return HEAD, TEXT, MONO


def stylesheet():
    return f"""
    QWidget {{ background:{WHITE}; color:{BLACK}; font-family:'{TEXT}'; font-size:15px; }}
    QFrame#card {{ background:{PANEL}; border-radius:14px; }}
    QFrame#card QLabel, QFrame#card QWidget#plain {{ background:transparent; }}
    QLabel#cardTitle {{ font-family:'{HEAD}'; font-weight:600; font-size:15px; color:{DARK_BLUE}; }}
    QLabel#caption {{ font-family:'{HEAD}'; font-weight:600; font-size:11px; color:{GREY}; }}
    QLabel#value {{ font-family:'{TEXT}'; font-size:16px; color:{BLACK}; }}
    QLabel#mono {{ font-family:'{MONO}'; font-size:13px; color:{GREY}; }}
    QLabel#title {{ font-family:'{HEAD}'; font-weight:600; font-size:28px; color:{BLACK}; }}
    QPlainTextEdit {{ background:transparent; border:none; font-family:'{MONO}'; font-size:12px; color:{DARK_BLUE}; }}
    QPushButton {{ font-family:'{HEAD}'; font-weight:600; font-size:13px; border-radius:8px; padding:8px 12px;
                   background:{WHITE}; color:{DARK_BLUE}; border:1px solid {GREY}; }}
    QPushButton:pressed {{ background:{OFF_WHITE}; }}
    QPushButton:disabled {{ background:transparent; color:{OFF_WHITE}; border:1px solid {OFF_WHITE}; }}
    QScrollBar:vertical {{ background:transparent; width:8px; margin:0; }}
    QScrollBar::handle:vertical {{ background:{OFF_WHITE}; border-radius:4px; min-height:30px; }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
    QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background:transparent; }}
    QToolTip {{ background:{DARK_BLUE}; color:{WHITE}; border:none; padding:6px; font-family:'{TEXT}'; }}
    QTabWidget::pane {{ border:none; }}
    QTabBar::tab {{ font-family:'{HEAD}'; font-weight:600; padding:10px 24px; background:{PANEL}; color:{GREY}; }}
    QTabBar::tab:selected {{ background:{DARK_BLUE}; color:{WHITE}; }}
    QMessageBox QPushButton {{ min-width:90px; }}
    """


def button_style(role, size=13):
    bg, fg = BUTTON[role]
    return (f"QPushButton {{background:{bg}; color:{fg}; border:none; font-size:{size}px;}}"
            f"QPushButton:pressed {{background:{BLACK};}}"
            f"QPushButton:disabled {{background:{PANEL}; color:{OFF_WHITE}; border:1px solid {OFF_WHITE};}}")


def chip_style(colors):
    bg, fg = colors
    return (f"background:{bg}; color:{fg}; border-radius:17px; padding:0 12px; font-family:'{HEAD}'; "
            'font-weight:600; font-size:12px;')


def logo(negative=False, width=230):
    """Primary logo as a crisp SVG widget (>= 175 px wide per the brand rules)."""
    w = QSvgWidget(LOGO_NEG if negative else LOGO)
    width = max(width, 175)
    w.setFixedSize(width, round(width * 49.37 / 195.84))       # SVG viewBox aspect
    w.setAttribute(Qt.WA_TranslucentBackground)
    w.setStyleSheet('background:transparent;')
    return w
