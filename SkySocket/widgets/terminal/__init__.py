"""
Terminal widget for user feedback and program status.

This widget is designed to give feedback on connection, test, and program
statuses. It does not dump everything from the terminal to the screen, but it
gives ample user feedback about the important aspects of the system with context
for user experience. It is used on both the simulation and testing tabs.

The SkyTerminal has not undergone any development. It should include but are not
limited to the following:
    - Realtime feedback for connection statuses (drone online, car connected,
        etc.).
    - Real time feedback on testing (Simulation finished. Log at log/location/,
        etc.).
    - Labels in a Bash style tag showing which device in the system the log is
        related to (drone@X.X.X.X, SkySocket, etc.).
    - Normal terminal feedback fro SSH connections/interactions.
    - User feedback for updated settings (Updated drone ip: X.X.X.X, running
        test with drone as server at ip X.X.X.X, etc.).
"""

from kivy.lang.builder import Builder
from kivy.uix.label import Label

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SkyTerminal>:
    id: terminal_placeholder
    bold: True
    padding: StyleConstants.terminal_padding, StyleConstants.terminal_padding
    pos_hint: StyleConstants.center_pos
    size_hint: 1, 0.2
    text: "TERMINAL PLACEHOLDER"
    canvas.before:
        Color:
            rgba: ColorConstants.blue
        Rectangle:
            pos: self.pos
            size: self.size
""")


class SkyTerminal(Label):
    """Terminal widget for user feedback about the system."""
