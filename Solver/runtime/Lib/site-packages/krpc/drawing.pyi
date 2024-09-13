from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable
from krpc.ui import UI
from krpc.spacecenter import SpaceCenter


class Drawing:

    def add_direction(self, direction, reference_frame, length=10.0, visible=True):
        # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame, float, bool) -> Drawing.Line
        """Draw a direction vector in the scene, starting from the origin of the given reference frame.
        :direction Direction to draw the line in.
        :reference_frame Reference frame that the direction is in and defines the start position.
        :length The length of the line.
        :visible Whether the line is visible."""
        ...

    def add_direction_from_com(self, direction, reference_frame, length=10.0, visible=True):
        # type: (Tuple[float,float,float], SpaceCenter.ReferenceFrame, float, bool) -> Drawing.Line
        """Draw a direction vector in the scene, from the center of mass of the active vessel.
        :direction Direction to draw the line in.
        :reference_frame Reference frame that the direction is in.
        :length The length of the line.
        :visible Whether the line is visible."""
        ...

    def add_line(self, start, end, reference_frame, visible=True):
        # type: (Tuple[float,float,float], Tuple[float,float,float], SpaceCenter.ReferenceFrame, bool) -> Drawing.Line
        """Draw a line in the scene.
        :start Position of the start of the line.
        :end Position of the end of the line.
        :reference_frame Reference frame that the positions are in.
        :visible Whether the line is visible."""
        ...

    def add_polygon(self, vertices, reference_frame, visible=True):
        # type: (List[Tuple[float,float,float]], SpaceCenter.ReferenceFrame, bool) -> Drawing.Polygon
        """Draw a polygon in the scene, defined by a list of vertices.
        :vertices Vertices of the polygon.
        :reference_frame Reference frame that the vertices are in.
        :visible Whether the polygon is visible."""
        ...

    def add_text(self, text, reference_frame, position, rotation, visible=True):
        # type: (str, SpaceCenter.ReferenceFrame, Tuple[float,float,float], Tuple[float,float,float,float], bool) -> Drawing.Text
        """Draw text in the scene.
        :text The string to draw.
        :reference_frame Reference frame that the text position is in.
        :position Position of the text.
        :rotation Rotation of the text, as a quaternion.
        :visible Whether the text is visible."""
        ...

    def clear(self, client_only=False):
        # type: (bool) -> None
        """Remove all objects being drawn.
        :client_only If true, only remove objects created by the calling client."""
        ...

    class Line:
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
        def end(self):
            # type: () -> Tuple[float,float,float]
            """End position of the line."""
            ...

        @end.setter
        def end(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def material(self):
            # type: () -> str
            """Material used to render the object.
            Creates the material from a shader with the given name."""
            ...

        @material.setter
        def material(self, value):
            # type: (str) -> None
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """Reference frame for the positions of the object."""
            ...

        @reference_frame.setter
        def reference_frame(self, value):
            # type: (SpaceCenter.ReferenceFrame) -> None
            ...

        @property
        def start(self):
            # type: () -> Tuple[float,float,float]
            """Start position of the line."""
            ...

        @start.setter
        def start(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def thickness(self):
            # type: () -> float
            """Set the thickness"""
            ...

        @thickness.setter
        def thickness(self, value):
            # type: (float) -> None
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the object."""
            ...

    class Polygon:
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
        def material(self):
            # type: () -> str
            """Material used to render the object.
            Creates the material from a shader with the given name."""
            ...

        @material.setter
        def material(self, value):
            # type: (str) -> None
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """Reference frame for the positions of the object."""
            ...

        @reference_frame.setter
        def reference_frame(self, value):
            # type: (SpaceCenter.ReferenceFrame) -> None
            ...

        @property
        def thickness(self):
            # type: () -> float
            """Set the thickness"""
            ...

        @thickness.setter
        def thickness(self, value):
            # type: (float) -> None
            ...

        @property
        def vertices(self):
            # type: () -> List[Tuple[float,float,float]]
            """Vertices for the polygon."""
            ...

        @vertices.setter
        def vertices(self, value):
            # type: (List[Tuple[float,float,float]]) -> None
            ...

        @property
        def visible(self):
            # type: () -> bool
            """Whether the object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the object."""
            ...

    class Text:
        @property
        def alignment(self):
            # type: () -> UI.TextAlignment
            """Alignment."""
            ...

        @alignment.setter
        def alignment(self, value):
            # type: (UI.TextAlignment) -> None
            ...

        @property
        def anchor(self):
            # type: () -> UI.TextAnchor
            """Anchor."""
            ...

        @anchor.setter
        def anchor(self, value):
            # type: (UI.TextAnchor) -> None
            ...

        @property
        def character_size(self):
            # type: () -> float
            """Character size."""
            ...

        @character_size.setter
        def character_size(self, value):
            # type: (float) -> None
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
        def material(self):
            # type: () -> str
            """Material used to render the object.
            Creates the material from a shader with the given name."""
            ...

        @material.setter
        def material(self, value):
            # type: (str) -> None
            ...

        @property
        def position(self):
            # type: () -> Tuple[float,float,float]
            """Position of the text."""
            ...

        @position.setter
        def position(self, value):
            # type: (Tuple[float,float,float]) -> None
            ...

        @property
        def reference_frame(self):
            # type: () -> SpaceCenter.ReferenceFrame
            """Reference frame for the positions of the object."""
            ...

        @reference_frame.setter
        def reference_frame(self, value):
            # type: (SpaceCenter.ReferenceFrame) -> None
            ...

        @property
        def rotation(self):
            # type: () -> Tuple[float,float,float,float]
            """Rotation of the text as a quaternion."""
            ...

        @rotation.setter
        def rotation(self, value):
            # type: (Tuple[float,float,float,float]) -> None
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
            """Whether the object is visible."""
            ...

        @visible.setter
        def visible(self, value):
            # type: (bool) -> None
            ...

        def remove(self):
            # type: () -> None
            """Remove the object."""
            ...

        @staticmethod
        def available_fonts():
            # type: () -> List[str]
            """A list of all available fonts."""
            ...
