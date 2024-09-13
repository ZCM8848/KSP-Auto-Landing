from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable


class UI:

    @property
    def stock_canvas(self):
        # type: () -> UI.Canvas
        """The stock UI canvas."""
        ...

    def add_canvas(self):
        # type: () -> UI.Canvas
        """Add a new canvas.

        If you want to add UI elements to KSPs stock UI canvas, use UI#stockCanvas."""
        ...

    def clear(self, client_only=False):
        # type: (bool) -> None
        """Remove all user interface elements.
        :client_only If true, only remove objects created by the calling client."""
        ...

    def message(self, content, duration=1.0, position=1, color=(1.0, 0.92, 0.016), size=20.0):
        # type: (str, float, UI.MessagePosition, Tuple[float,float,float], float) -> None
        """Display a message on the screen.

        The message appears just like a stock message, for example quicksave or quickload messages.
        :content Message content.
        :duration Duration before the message disappears, in seconds.
        :position Position to display the message.
        :size Size of the message, differs per position.
        :color The color of the message."""
        ...

    class Button:
        @property
        def clicked(self):
            # type: () -> bool
            """Whether the button has been clicked.

            This property is set to true when the user clicks the button.
            A client script should reset the property to false in order to detect subsequent button presses."""
            ...

        @clicked.setter
        def clicked(self, value):
            # type: (bool) -> None
            ...

        @property
        def rect_transform(self):
            # type: () -> UI.RectTransform
            """The rect transform for the text."""
            ...

        @property
        def text(self):
            # type: () -> UI.Text
            """The text for the button."""
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the UI object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the UI object."""
            ...

    class Canvas:
        @property
        def rect_transform(self):
            # type: () -> UI.RectTransform
            """The rect transform for the canvas."""
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the UI object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def add_button(self, content, visible=True):
            # type: (str, bool) -> UI.Button
            """Add a button to the canvas.
            :content The label for the button.
            :visible Whether the button is visible."""
            ...

        def add_input_field(self, visible=True):
            # type: (bool) -> UI.InputField
            """Add an input field to the canvas.
            :visible Whether the input field is visible."""
            ...

        def add_panel(self, visible=True):
            # type: (bool) -> UI.Panel
            """Create a new container for user interface elements.
            :visible Whether the panel is visible."""
            ...

        def add_text(self, content, visible=True):
            # type: (str, bool) -> UI.Text
            """Add text to the canvas.
            :content The text.
            :visible Whether the text is visible."""
            ...

        def remove(self):
            # type: () -> None
            """Remove the UI object."""
            ...

    class InputField:
        @property
        def changed(self):
            # type: () -> bool
            """Whether the input field has been changed.

            This property is set to true when the user modifies the value of the input field.
            A client script should reset the property to false in order to detect subsequent changes."""
            ...

        @changed.setter
        def changed(self, value):
            # type: (bool) -> None
            ...

        @property
        def rect_transform(self):
            # type: () -> UI.RectTransform
            """The rect transform for the input field."""
            ...

        @property
        def text(self):
            # type: () -> UI.Text
            """The text component of the input field.

            Use UI.InputField#value to get and set the value in the field.
            This object can be used to alter the style of the input field's text."""
            ...

        @property
        def value(self):
            # type: () -> str
            """The value of the input field."""
            ...

        @value.setter
        def value(self, value):
            # type: (str) -> None
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the UI object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the UI object."""
            ...

    class Panel:
        @property
        def rect_transform(self):
            # type: () -> UI.RectTransform
            """The rect transform for the panel."""
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the UI object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def add_button(self, content, visible=True):
            # type: (str, bool) -> UI.Button
            """Add a button to the panel.
            :content The label for the button.
            :visible Whether the button is visible."""
            ...

        def add_input_field(self, visible=True):
            # type: (bool) -> UI.InputField
            """Add an input field to the panel.
            :visible Whether the input field is visible."""
            ...

        def add_panel(self, visible=True):
            # type: (bool) -> UI.Panel
            """Create a panel within this panel.
            :visible Whether the new panel is visible."""
            ...

        def add_text(self, content, visible=True):
            # type: (str, bool) -> UI.Text
            """Add text to the panel.
            :content The text.
            :visible Whether the text is visible."""
            ...

        def remove(self):
            # type: () -> None
            """Remove the UI object."""
            ...

    class RectTransform:
        def anchor(self, value):
            # type: (Tuple[float,float]) -> None
            """Set the minimum and maximum anchor points as a fraction of the size of the parent rectangle."""
            ...
        anchor = property(None, anchor)

        @property
        def anchor_max(self):
            # type: () -> Tuple[float,float]
            """The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle."""
            ...

        @anchor_max.setter
        def anchor_max(self, value):
            # type: (Tuple[float,float]) -> None
            ...

        @property
        def anchor_min(self):
            # type: () -> Tuple[float,float]
            """The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle."""
            ...

        @anchor_min.setter
        def anchor_min(self, value):
            # type: (Tuple[float,float]) -> None
            ...

        @property
        def local_position(self):
            # type: () -> Tuple[float,float,float]
            """Position of the rectangles pivot point relative to the anchors."""
            ...

        @local_position.setter
        def local_position(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def lower_left(self):
            # type: () -> Tuple[float,float]
            """Position of the rectangles lower left corner relative to the anchors."""
            ...

        @lower_left.setter
        def lower_left(self, value):
            # type: (Tuple[float,float]) -> None
            ...

        @property
        def pivot(self):
            # type: () -> Tuple[float,float]
            """Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself."""
            ...

        @pivot.setter
        def pivot(self, value):
            # type: (Tuple[float,float]) -> None
            ...

        @property
        def position(self):
            # type: () -> Tuple[float,float]
            """Position of the rectangles pivot point relative to the anchors."""
            ...

        @position.setter
        def position(self, value):
            # type: (Tuple[float,float]) -> None
            ...

        @property
        def rotation(self):
            # type: () -> Tuple[float,float,float,float]
            """Rotation, as a quaternion, of the object around its pivot point."""
            ...

        @rotation.setter
        def rotation(self, value):
            # type: (Tuple[float,float,float,float]) -> None
            ...

        @property
        def scale(self):
            # type: () -> Tuple[float,float,float]
            """Scale factor applied to the object in the x, y and z dimensions."""
            ...

        @scale.setter
        def scale(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def size(self):
            # type: () -> Tuple[float,float]
            """Width and height of the rectangle."""
            ...

        @size.setter
        def size(self, value):
            # type: (Tuple[float,float]) -> None
            ...

        @property
        def upper_right(self):
            # type: () -> Tuple[float,float]
            """Position of the rectangles upper right corner relative to the anchors."""
            ...

        @upper_right.setter
        def upper_right(self, value):
            # type: (Tuple[float,float]) -> None
            ...

    class Text:
        @property
        def alignment(self):
            # type: () -> UI.TextAnchor
            """Alignment."""
            ...

        @alignment.setter
        def alignment(self, value):
            # type: (UI.TextAnchor) -> None
            ...

        @property
        def available_fonts(self):
            # type: () -> List[str]
            """A list of all available fonts."""
            ...

        @property
        def color(self):
            # type: () -> Tuple[float,float,float]
            """Set the color"""
            ...

        @color.setter
        def color(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def content(self):
            # type: () -> str
            """The text string"""
            ...

        @content.setter
        def content(self, value):
            # type: (str) -> None
            ...

        @property
        def font(self):
            # type: () -> str
            """Name of the font"""
            ...

        @font.setter
        def font(self, value):
            # type: (str) -> None
            ...

        @property
        def line_spacing(self):
            # type: () -> float
            """Line spacing."""
            ...

        @line_spacing.setter
        def line_spacing(self, value):
            # type: (float) -> None
            ...

        @property
        def rect_transform(self):
            # type: () -> UI.RectTransform
            """The rect transform for the text."""
            ...

        @property
        def size(self):
            # type: () -> int
            """Font size."""
            ...

        @size.setter
        def size(self, value):
            # type: (int) -> None
            ...

        @property
        def style(self):
            # type: () -> UI.FontStyle
            """Font style."""
            ...

        @style.setter
        def style(self, value):
            # type: (UI.FontStyle) -> None
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the UI object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the UI object."""
            ...

    class FontStyle:
        """Font style."""
        normal = 0
        bold = 1
        italic = 2
        bold_and_italic = 3

    class MessagePosition:
        """Message position."""
        bottom_center = 0
        top_center = 1
        top_left = 2
        top_right = 3

    class TextAlignment:
        """Text alignment."""
        left = 0
        right = 1
        center = 2

    class TextAnchor:
        """Text alignment."""
        lower_center = 0
        lower_left = 1
        lower_right = 2
        middle_center = 3
        middle_left = 4
        middle_right = 5
        upper_center = 6
        upper_left = 7
        upper_right = 8
