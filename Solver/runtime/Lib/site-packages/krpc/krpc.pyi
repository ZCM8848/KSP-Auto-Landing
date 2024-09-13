from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable


class KRPC:

    @property
    def clients(self):
        # type: () -> List[Tuple[bytes,str,str]]
        """A list of RPC clients that are currently connected to the server.
        Each entry in the list is a clients identifier, name and address."""
        ...

    @property
    def current_game_scene(self):
        # type: () -> KRPC.GameScene
        """Get the current game scene."""
        ...

    @property
    def paused(self):
        # type: () -> bool
        """Whether the game is paused."""
        ...

    @paused.setter
    def paused(self, value):
        # type: (bool) -> None
        ...

    def add_event(self, expression):
        # type: (KRPC.Expression) -> krpc.schema.KRPC.Event
        """Create an event from a server side expression."""
        ...

    def add_stream(self, call, start=True):
        # type: (krpc.schema.KRPC.ProcedureCall, bool) -> krpc.schema.KRPC.Stream
        """Add a streaming request and return its identifier."""
        ...

    def get_client_id(self):
        # type: () -> bytes
        """Returns the identifier for the current client."""
        ...

    def get_client_name(self):
        # type: () -> str
        """Returns the name of the current client.
        This is an empty string if the client has no name."""
        ...

    def get_services(self):
        # type: () -> krpc.schema.KRPC.Services
        """Returns information on all services, procedures, classes, properties etc. provided by the server.
        Can be used by client libraries to automatically create functionality such as stubs."""
        ...

    def get_status(self):
        # type: () -> krpc.schema.KRPC.Status
        """Returns some information about the server, such as the version."""
        ...

    def remove_stream(self, id):
        # type: (int) -> None
        """Remove a streaming request."""
        ...

    def set_stream_rate(self, id, rate):
        # type: (int, float) -> None
        """Set the update rate for a stream in Hz."""
        ...

    def start_stream(self, id):
        # type: (int) -> None
        """Start a previously added streaming request."""
        ...

    class Expression:
        @staticmethod
        def add(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Numerical addition.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def aggregate(, arg, func):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Applies an accumulator function over a sequence.
            @return The accumulated value.
            :arg The collection.
            :func The accumulator function."""
            ...

        @staticmethod
        def aggregate_with_seed(, arg, seed, func):
            # type: (KRPC.Expression, KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Applies an accumulator function over a sequence, with a given seed.
            @return The accumulated value.
            :arg The collection.
            :seed The seed value.
            :func The accumulator function."""
            ...

        @staticmethod
        def all(, arg, predicate):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Determine whether all items in a collection satisfy a boolean predicate.
            @return Whether all items satisfy the predicate.
            :arg The collection.
            :predicate The predicate function."""
            ...

        @staticmethod
        def and(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Boolean and operator.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def any(, arg, predicate):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Determine whether any item in a collection satisfies a boolean predicate.
            @return Whether any item satisfies the predicate.
            :arg The collection.
            :predicate The predicate function."""
            ...

        @staticmethod
        def average(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Minimum of all elements in a collection.
            @return The minimum elements in the collection.
            :arg The list or set."""
            ...

        @staticmethod
        def call(, call):
            # type: (krpc.schema.KRPC.ProcedureCall) -> KRPC.Expression
            """An RPC call.
            :call """
            ...

        @staticmethod
        def cast(, arg, type):
            # type: (KRPC.Expression, KRPC.Type) -> KRPC.Expression
            """Perform a cast to the given type.
            :arg 
            :type Type to cast the argument to."""
            ...

        @staticmethod
        def concat(, arg1, arg2):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Concatenate two sequences.
            @return The first sequence followed by the second sequence.
            :arg1 The first sequence.
            :arg2 The second sequence."""
            ...

        @staticmethod
        def constant_bool(, value):
            # type: (bool) -> KRPC.Expression
            """A constant value of boolean type.
            :value """
            ...

        @staticmethod
        def constant_double(, value):
            # type: (float) -> KRPC.Expression
            """A constant value of double precision floating point type.
            :value """
            ...

        @staticmethod
        def constant_float(, value):
            # type: (float) -> KRPC.Expression
            """A constant value of single precision floating point type.
            :value """
            ...

        @staticmethod
        def constant_int(, value):
            # type: (int) -> KRPC.Expression
            """A constant value of integer type.
            :value """
            ...

        @staticmethod
        def constant_string(, value):
            # type: (str) -> KRPC.Expression
            """A constant value of string type.
            :value """
            ...

        @staticmethod
        def contains(, arg, value):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Determine if a collection contains a value.
            @return Whether the collection contains a value.
            :arg The collection.
            :value The value to look for."""
            ...

        @staticmethod
        def count(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Number of elements in a collection.
            @return The number of elements in the collection.
            :arg The list, set or dictionary."""
            ...

        @staticmethod
        def create_dictionary(, keys, values):
            # type: (List[KRPC.Expression], List[KRPC.Expression]) -> KRPC.Expression
            """Construct a dictionary, from a list of corresponding keys and values.
            @return The dictionary.
            :keys The keys. Should all be of the same type.
            :values The values. Should all be of the same type."""
            ...

        @staticmethod
        def create_list(, values):
            # type: (List[KRPC.Expression]) -> KRPC.Expression
            """Construct a list.
            @return The list.
            :values The value. Should all be of the same type."""
            ...

        @staticmethod
        def create_set(, values):
            # type: (Set[KRPC.Expression]) -> KRPC.Expression
            """Construct a set.
            @return The set.
            :values The values. Should all be of the same type."""
            ...

        @staticmethod
        def create_tuple(, elements):
            # type: (List[KRPC.Expression]) -> KRPC.Expression
            """Construct a tuple.
            @return The tuple.
            :elements The elements."""
            ...

        @staticmethod
        def divide(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Numerical division.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def equal(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Equality comparison.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def exclusive_or(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Boolean exclusive-or operator.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def function(, parameters, body):
            # type: (List[KRPC.Expression], KRPC.Expression) -> KRPC.Expression
            """A function.
            @return A function.
            :parameters The parameters of the function.
            :body The body of the function."""
            ...

        @staticmethod
        def get(, arg, index):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Access an element in a tuple, list or dictionary.
            @return The element.
            :arg The tuple, list or dictionary.
            :index The index of the element to access.
            A zero indexed integer for a tuple or list, or a key for a dictionary."""
            ...

        @staticmethod
        def greater_than(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Greater than numerical comparison.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def greater_than_or_equal(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Greater than or equal numerical comparison.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def invoke(, function, args):
            # type: (KRPC.Expression, Dict[str, KRPC.Expression]) -> KRPC.Expression
            """A function call.
            @return A function call.
            :function The function to call.
            :args The arguments to call the function with."""
            ...

        @staticmethod
        def left_shift(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Bitwise left shift.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def less_than(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Less than numerical comparison.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def less_than_or_equal(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Less than or equal numerical comparison.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def max(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Maximum of all elements in a collection.
            @return The maximum elements in the collection.
            :arg The list or set."""
            ...

        @staticmethod
        def min(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Minimum of all elements in a collection.
            @return The minimum elements in the collection.
            :arg The list or set."""
            ...

        @staticmethod
        def modulo(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Numerical modulo operator.
            :arg0 
            :arg1 
            @return The remainder of arg0 divided by arg1"""
            ...

        @staticmethod
        def multiply(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Numerical multiplication.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def not(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Boolean negation operator.
            :arg """
            ...

        @staticmethod
        def not_equal(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Inequality comparison.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def or(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Boolean or operator.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def order_by(, arg, key):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Order a collection using a key function.
            @return The ordered collection.
            :arg The collection to order.
            :key A function that takes a value from the collection and generates a key to sort on."""
            ...

        @staticmethod
        def parameter(, name, type):
            # type: (str, KRPC.Type) -> KRPC.Expression
            """A named parameter of type double.
            @return A named parameter.
            :name The name of the parameter.
            :type The type of the parameter."""
            ...

        @staticmethod
        def power(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Numerical power operator.
            :arg0 
            :arg1 
            @return arg0 raised to the power of arg1, with type of arg0"""
            ...

        @staticmethod
        def right_shift(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Bitwise right shift.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def select(, arg, func):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Run a function on every element in the collection.
            @return The modified collection.
            :arg The list or set.
            :func The function."""
            ...

        @staticmethod
        def subtract(, arg0, arg1):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Numerical subtraction.
            :arg0 
            :arg1 """
            ...

        @staticmethod
        def sum(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Sum all elements of a collection.
            @return The sum of the elements in the collection.
            :arg The list or set."""
            ...

        @staticmethod
        def to_list(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Convert a collection to a list.
            @return The collection as a list.
            :arg The collection."""
            ...

        @staticmethod
        def to_set(, arg):
            # type: (KRPC.Expression) -> KRPC.Expression
            """Convert a collection to a set.
            @return The collection as a set.
            :arg The collection."""
            ...

        @staticmethod
        def where(, arg, func):
            # type: (KRPC.Expression, KRPC.Expression) -> KRPC.Expression
            """Run a function on every element in the collection.
            @return The modified collection.
            :arg The list or set.
            :func The function."""
            ...

    class Type:
        @staticmethod
        def bool():
            # type: () -> KRPC.Type
            """Bool type."""
            ...

        @staticmethod
        def double():
            # type: () -> KRPC.Type
            """Double type."""
            ...

        @staticmethod
        def float():
            # type: () -> KRPC.Type
            """Float type."""
            ...

        @staticmethod
        def int():
            # type: () -> KRPC.Type
            """Int type."""
            ...

        @staticmethod
        def string():
            # type: () -> KRPC.Type
            """String type."""
            ...

    class GameScene:
        """The game scene. See KRPC#currentGameScene."""
        space_center = 0
        flight = 1
        tracking_station = 2
        editor_vab = 3
        editor_sph = 4
