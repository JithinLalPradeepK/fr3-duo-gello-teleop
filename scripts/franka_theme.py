"""Franka Robotics brand theme for the demo GUI (Brand Guidelines, February 2027).

Two modes, LIGHT (default, white background) and DARK. Brand rules applied:
- Colours: Franka Black #000000, White #FFFFFF, Dark Blue #1B2F36, Franka Grey #587380, Off-White #D4DBDC.
  Accent colours only as tiny lights (status dots). EVERY BUTTON uses Franka Grey as background.
- Fonts: Montserrat (assets/brand/fonts, SIL OFL) for everything; DejaVu Sans Mono stands in for RM Mono.
- Logo: pure black on light, pure white on dark (assets/brand/logo_black.png / logo_white.png, transparent),
  never narrower than 175 px, top-left, >= 48 px from the side.
Functional status colours (green = connected, red = problem, amber = working) are kept only for the small
status dots and the error bar - they are not brand colours on purpose.
"""
import os

from PyQt5.QtGui import QFontDatabase, QPixmap

DEMO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # repo root (works from any checkout)
ASSETS = os.path.join(DEMO, 'assets', 'brand')
FONT_DIR = os.path.join(ASSETS, 'fonts')
LOGO_BLACK = os.path.join(ASSETS, 'logo_black.png')
LOGO_WHITE = os.path.join(ASSETS, 'logo_white.png')
ROBOTS = os.path.join(ASSETS, 'fr3_duo.png')                  # trimmed from 'FR3 Duo.transparent.png'
ICON_BLACK = os.path.join(ASSETS, 'FrankaRobotics_Trademark_Digital_RGB.png')
ICON_WHITE = os.path.join(ASSETS, 'FrankaRobotics_Trademark_negative_Digital_RGB.png')

# brand palette
BLACK, WHITE = '#000000', '#FFFFFF'
DARK_BLUE, GREY, OFF_WHITE = '#1B2F36', '#587380', '#D4DBDC'
GREY_LIGHT = '#E8ECED'                  # light shade of Franka Grey (disconnected chips)
GREY_HOVER = '#6B8694'                  # Franka Grey lightened (dark-mode hover)
# functional status colours (tiny dots / error bar only)
SAFE_GREEN, SAFE_RED, SAFE_AMBER = '#2E9E5B', '#D32F2F', '#E69500'

HEAD = TEXT = 'Montserrat'
MONO = 'DejaVu Sans Mono'
MINT = '#75C28B'                        # Franka Mint: switch tracks (on)
MODE = 'light'                          # current mode, set by the GUI (light is the default)

# per-mode palette
PALETTES = {
    'light': dict(bg=WHITE, fg=BLACK, muted=GREY, panel='#F3F5F5', line=OFF_WHITE, nav=WHITE,
                  chip_ok_bg=DARK_BLUE, chip_ok_fg=WHITE, chip_bad_bg=GREY_LIGHT, chip_bad_fg=GREY,
                  btn=GREY, btn_hover=DARK_BLUE, btn_checked=DARK_BLUE, btn_dis_bg=GREY_LIGHT, btn_dis_fg='#9AABB3',
                  popup=DARK_BLUE, popup_fg=WHITE, camera_bg='#EDF0F1', row=WHITE),
    'dark': dict(bg='#13222A', fg=WHITE, muted=OFF_WHITE, panel='#1B2F36', line='#2E4650', nav='#13222A',
                 chip_ok_bg='#0B151A', chip_ok_fg=WHITE, chip_bad_bg='#2A3E47', chip_bad_fg=OFF_WHITE,
                 btn=GREY, btn_hover=GREY_HOVER, btn_checked=BLACK, btn_dis_bg='#22363F', btn_dis_fg='#6F848D',
                 popup='#0B151A', popup_fg=WHITE, camera_bg='#0E1A1E', row='#243B45'),
}


def pal():
    return PALETTES[MODE]


def load_fonts():
    """Register the brand fonts shipped in assets/brand/fonts."""
    global HEAD, TEXT, MONO
    if os.path.isdir(FONT_DIR):
        for f in sorted(os.listdir(FONT_DIR)):
            if f.lower().endswith(('.ttf', '.otf')):
                QFontDatabase.addApplicationFont(os.path.join(FONT_DIR, f))
    fams = set(QFontDatabase().families())
    HEAD = TEXT = 'Montserrat' if 'Montserrat' in fams else 'DejaVu Sans'
    MONO = next((f for f in ('RM Mono', 'DejaVu Sans Mono') if f in fams), 'Monospace')


def stylesheet(mode='light'):
    c = PALETTES[mode]
    return f"""
    QWidget {{ background:{c['bg']}; color:{c['fg']}; font-family:'{TEXT}'; font-size:15px; }}
    QFrame#navbar {{ background:{c['nav']}; border-bottom:1px solid {c['line']}; }}
    QFrame#navbar QLabel, QFrame#navbar QWidget#plain {{ background:transparent; }}
    QFrame#card {{ background:{c['panel']}; border-radius:14px; }}
    QFrame#card QLabel, QFrame#card QWidget#plain {{ background:transparent; }}
    QLabel#plain {{ background:transparent; }}
    QLabel#cardTitle {{ font-family:'{HEAD}'; font-weight:600; font-size:15px; color:{c['fg']}; }}
    QLabel#caption {{ font-family:'{HEAD}'; font-weight:600; font-size:11px; color:{c['muted']}; }}
    QLabel#value {{ font-size:15px; }}
    QLabel#mono {{ font-family:'{MONO}'; font-size:12px; color:{c['muted']}; }}
    QLabel#headline {{ font-family:'{HEAD}'; font-weight:600; font-size:56px; color:{c['fg']}; background:transparent; }}
    QLabel#subline {{ font-family:'{HEAD}'; font-weight:500; font-size:15px; color:{c['muted']}; background:transparent; }}
    QLabel#pageTitle {{ font-family:'{HEAD}'; font-weight:600; font-size:24px; background:transparent; }}
    QPlainTextEdit {{ background:transparent; border:none; font-family:'{MONO}'; font-size:12px; color:{c['fg']}; }}
    QPushButton {{ font-family:'{HEAD}'; font-weight:600; font-size:13px; border-radius:8px; padding:9px 16px;
                   background:{c['btn']}; color:{WHITE}; border:none; }}
    QPushButton:hover {{ background:{c['btn_hover']}; }}
    QPushButton:pressed {{ background:{BLACK}; }}
    QPushButton:checked {{ background:{c['btn_checked']}; }}
    QPushButton:disabled {{ background:{c['btn_dis_bg']}; color:{c['btn_dis_fg']}; }}
    QPushButton#stop {{ border:2px solid {SAFE_RED}; }}
    QPushButton#navbtn {{ padding:9px 18px; }}
    QPushButton::menu-indicator {{ image:none; width:0; }}
    QMenu {{ background:{c['panel']}; color:{c['fg']}; border:1px solid {c['line']}; padding:6px; }}
    QMenu::item {{ padding:9px 22px; border-radius:6px; font-family:'{HEAD}'; font-weight:500; font-size:13px; }}
    QMenu::item:selected {{ background:{GREY}; color:{WHITE}; }}
    QMenu::separator {{ height:1px; background:{c['line']}; margin:6px 4px; }}
    QScrollBar:vertical {{ background:transparent; width:8px; margin:0; }}
    QScrollBar::handle:vertical {{ background:{c['line']}; border-radius:4px; min-height:30px; }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
    QToolTip {{ background:{c['popup']}; color:{c['popup_fg']}; border:none; padding:6px; }}
    QMessageBox QPushButton {{ min-width:90px; }}
    """


def logo_pixmap(mode, width=220):
    """Black logo on light, white logo on dark (transparent PNGs). Width >= 175 px per the brand rules."""
    width = max(width, 175)
    pm = QPixmap(LOGO_WHITE if mode == 'dark' else LOGO_BLACK)
    return pm, width, round(width * pm.height() / max(pm.width(), 1))


def icon_pixmap(mode):
    """Franka trademark (running girl): black on light, white on dark. Shown where a camera picture is missing."""
    return QPixmap(ICON_WHITE if mode == 'dark' else ICON_BLACK)
