# Expressions

class Expression
:   A server side expression.

    static constant\_double(*value*)
    :   A constant value of double precision floating point type.

        Parameters:
        :   **value** (*float*)

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_float(*value*)
    :   A constant value of single precision floating point type.

        Parameters:
        :   **value** (*float*)

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_int(*value*)
    :   A constant value of integer type.

        Parameters:
        :   **value** (*int*)

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_bool(*value*)
    :   A constant value of boolean type.

        Parameters:
        :   **value** (*bool*)

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_string(*value*)
    :   A constant value of string type.

        Parameters:
        :   **value** (*str*)

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static call(*call*)
    :   An RPC call.

        Parameters:
        :   **call** (*krpc.schema.KRPC.ProcedureCall*)

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static equal(*arg0*, *arg1*)
    :   Equality comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static not\_equal(*arg0*, *arg1*)
    :   Inequality comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static greater\_than(*arg0*, *arg1*)
    :   Greater than numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static greater\_than\_or\_equal(*arg0*, *arg1*)
    :   Greater than or equal numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static less\_than(*arg0*, *arg1*)
    :   Less than numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static less\_than\_or\_equal(*arg0*, *arg1*)
    :   Less than or equal numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static and\_(*arg0*, *arg1*)
    :   Boolean and operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static or\_(*arg0*, *arg1*)
    :   Boolean or operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static exclusive\_or(*arg0*, *arg1*)
    :   Boolean exclusive-or operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static not\_(*arg*)
    :   Boolean negation operator.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static add(*arg0*, *arg1*)
    :   Numerical addition.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static subtract(*arg0*, *arg1*)
    :   Numerical subtraction.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static multiply(*arg0*, *arg1*)
    :   Numerical multiplication.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static divide(*arg0*, *arg1*)
    :   Numerical division.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static modulo(*arg0*, *arg1*)
    :   Numerical modulo operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Returns:
        :   The remainder of arg0 divided by arg1

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static power(*arg0*, *arg1*)
    :   Numerical power operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Returns:
        :   arg0 raised to the power of arg1, with type of arg0

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static left\_shift(*arg0*, *arg1*)
    :   Bitwise left shift.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static right\_shift(*arg0*, *arg1*)
    :   Bitwise right shift.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression"))

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static cast(*arg*, *type*)
    :   Perform a cast to the given type.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression"))
            - **type** ([*Type*](#KRPC.Type "KRPC.Type")) – Type to cast the argument to.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static parameter(*name*, *type*)
    :   A named parameter of type double.

        Parameters:
        :   - **name** (*str*) – The name of the parameter.
            - **type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the parameter.

        Returns:
        :   A named parameter.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static function(*parameters*, *body*)
    :   A function.

        Parameters:
        :   - **parameters** (*list*) – The parameters of the function.
            - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The body of the function.

        Returns:
        :   A function.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static invoke(*function*, *args*)
    :   A function call.

        Parameters:
        :   - **function** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function to call.
            - **args** (*dict*) – The arguments to call the function with.

        Returns:
        :   A function call.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_tuple(*elements*)
    :   Construct a tuple.

        Parameters:
        :   **elements** (*list*) – The elements.

        Returns:
        :   The tuple.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_list(*values*)
    :   Construct a list.

        Parameters:
        :   **values** (*list*) – The value. Should all be of the same type.

        Returns:
        :   The list.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_set(*values*)
    :   Construct a set.

        Parameters:
        :   **values** (*set*) – The values. Should all be of the same type.

        Returns:
        :   The set.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_dictionary(*keys*, *values*)
    :   Construct a dictionary, from a list of corresponding keys and values.

        Parameters:
        :   - **keys** (*list*) – The keys. Should all be of the same type.
            - **values** (*list*) – The values. Should all be of the same type.

        Returns:
        :   The dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static to\_list(*arg*)
    :   Convert a collection to a list.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The collection as a list.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static to\_set(*arg*)
    :   Convert a collection to a set.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The collection as a set.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static get(*arg*, *index*)
    :   Access an element in a tuple, list or dictionary.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The tuple, list or dictionary.
            - **index** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The index of the element to access. A zero indexed integer for a tuple or list, or a key for a dictionary.

        Returns:
        :   The element.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static count(*arg*)
    :   Number of elements in a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list, set or dictionary.

        Returns:
        :   The number of elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static sum(*arg*)
    :   Sum all elements of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The sum of the elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static max(*arg*)
    :   Maximum of all elements in a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The maximum elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static min(*arg*)
    :   Minimum of all elements in a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The minimum elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static average(*arg*)
    :   Minimum of all elements in a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The minimum elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static select(*arg*, *func*)
    :   Run a function on every element in the collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function.

        Returns:
        :   The modified collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static where(*arg*, *func*)
    :   Run a function on every element in the collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function.

        Returns:
        :   The modified collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static contains(*arg*, *value*)
    :   Determine if a collection contains a value.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to look for.

        Returns:
        :   Whether the collection contains a value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static aggregate(*arg*, *func*)
    :   Applies an accumulator function over a sequence.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The accumulator function.

        Returns:
        :   The accumulated value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static aggregate\_with\_seed(*arg*, *seed*, *func*)
    :   Applies an accumulator function over a sequence, with a given seed.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **seed** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The seed value.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The accumulator function.

        Returns:
        :   The accumulated value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static concat(*arg1*, *arg2*)
    :   Concatenate two sequences.

        Parameters:
        :   - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first sequence.
            - **arg2** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second sequence.

        Returns:
        :   The first sequence followed by the second sequence.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static order\_by(*arg*, *key*)
    :   Order a collection using a key function.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection to order.
            - **key** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – A function that takes a value from the collection and generates a key to sort on.

        Returns:
        :   The ordered collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static all(*arg*, *predicate*)
    :   Determine whether all items in a collection satisfy a boolean predicate.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **predicate** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The predicate function.

        Returns:
        :   Whether all items satisfy the predicate.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static any(*arg*, *predicate*)
    :   Determine whether any item in a collection satisfies a boolean predicate.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **predicate** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The predicate function.

        Returns:
        :   Whether any item satisfies the predicate.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

class Type
:   A server side expression.

    static double()
    :   Double type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static float()
    :   Float type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static int()
    :   Int type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static bool()
    :   Bool type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static string()
    :   String type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")
