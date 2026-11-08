# Expressions

The nodes a server side function is built from, and the types they work with. See the
[server side functions tutorial](https://krpc.github.io/krpc/latest/tutorials/server-side-functions.html#tutorial-server-side-functions) for how a function is
assembled and run.

class Expression
:   A server side expression.

    > **Note**
    >
    > Numeric operands of differing types are converted to a common type.

    return\_type
    :   The type of the value the expression evaluates to. Throws an exception if the
        value cannot be sent to a client.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

        > **Note**
        >
        > A function created with [`Expression.lambda_()`](#KRPC.Expression.lambda_ "KRPC.Expression.lambda_") evaluates to the value it returns. A
        > lazily evaluated collection, such as from [`Expression.select()`](#KRPC.Expression.select "KRPC.Expression.select"), is sent once
        > converted by [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") or [`Expression.to_set()`](#KRPC.Expression.to_set "KRPC.Expression.to_set").

    has\_return\_type
    :   Whether the expression evaluates to a value.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        > **Note**
        >
        > An expression with no value has no [`Expression.return_type`](#KRPC.Expression.return_type "KRPC.Expression.return_type"). An empty
        > collection and no value both encode to an empty sequence of bytes.

    parameter\_types
    :   The types of the function’s parameters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Type`](#KRPC.Type "KRPC.Type"))

        > **Note**
        >
        > Empty for a function that is not created with [`Expression.lambda_()`](#KRPC.Expression.lambda_ "KRPC.Expression.lambda_").

    static constant\_double(*value*)
    :   A constant value of double precision floating point type.

        Parameters:
        :   **value** (*float*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_float(*value*)
    :   A constant value of single precision floating point type.

        Parameters:
        :   **value** (*float*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_int(*value*)
    :   A constant value of integer type.

        Parameters:
        :   **value** (*int*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_long(*value*)
    :   A constant value of long integer type.

        Parameters:
        :   **value** (*int*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_u\_int(*value*)
    :   A constant value of unsigned integer type.

        Parameters:
        :   **value** (*int*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_u\_long(*value*)
    :   A constant value of unsigned long integer type.

        Parameters:
        :   **value** (*int*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_bool(*value*)
    :   A constant value of boolean type.

        Parameters:
        :   **value** (*bool*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_string(*value*)
    :   A constant value of string type.

        Parameters:
        :   **value** (*str*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_bytes(*value*)
    :   A constant value of bytes type.

        Parameters:
        :   **value** (*bytes*) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_enum(*service*, *name*, *value*)
    :   A constant value of an enumeration a service defines.

        Parameters:
        :   - **service** (*str*) – The name of the service the enumeration is defined in.
            - **name** (*str*) – The name of the enumeration.
            - **value** (*int*) – The value of the member of the enumeration.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static constant\_null(*type*)
    :   A null constant of the given type.

        Parameters:
        :   **type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The type is made nullable, as by [`Type.nullable_type()`](#KRPC.Type.nullable_type "KRPC.Type.nullable_type").

    static constant\_object(*value*)
    :   A constant object, an instance of a class a service defines.

        Parameters:
        :   **value** (*int*) – The object identifier of the object.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The object is given by its object identifier.

    static call(*call*)
    :   An RPC call.

        Parameters:
        :   **call** (*krpc.schema.KRPC.ProcedureCall*) – The procedure call.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The instance the call is made on, and the values of its arguments, are fixed
        > when the expression is created. A call to a procedure with no return value is
        > evaluated for its effects, as a statement in a [`Expression.block()`](#KRPC.Expression.block "KRPC.Expression.block"). A collection the
        > procedure returns is copied, as a client would receive it.

    static call\_with\_arguments(*call*, *args*)
    :   An RPC call, where some or all of the arguments are computed by expressions.

        Parameters:
        :   - **call** (*krpc.schema.KRPC.ProcedureCall*) – The RPC to call.
            - **args** (*dict*) – Expressions computing the call’s arguments, by position.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Position 0 is the instance of a class method or property. A position with no
        > expression takes the argument encoded in the call, or the parameter’s default. A
        > collection the procedure returns is copied, as a client would receive it.

    static equal(*arg0*, *arg1*)
    :   Equality comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Collections, tuples, bytes and structures compare by their contents. Throws an
        > exception if the operands are of unrelated types.

    static not\_equal(*arg0*, *arg1*)
    :   Inequality comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Collections, tuples, bytes and structures compare by their contents. Throws an
        > exception if the operands are of unrelated types.

    static greater\_than(*arg0*, *arg1*)
    :   Greater than numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static greater\_than\_or\_equal(*arg0*, *arg1*)
    :   Greater than or equal numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static less\_than(*arg0*, *arg1*)
    :   Less than numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static less\_than\_or\_equal(*arg0*, *arg1*)
    :   Less than or equal numerical comparison.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static is\_null(*value*)
    :   Whether a value is null. Throws an exception if the type of the value cannot be null.

        Parameters:
        :   **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static and\_(*arg0*, *arg1*)
    :   Logical or bitwise and, which evaluates both operands.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static or\_(*arg0*, *arg1*)
    :   Logical or bitwise or, which evaluates both operands.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static conditional\_and(*arg0*, *arg1*)
    :   Logical and, which evaluates the second operand only when the first is true.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static conditional\_or(*arg0*, *arg1*)
    :   Logical or, which evaluates the second operand only when the first is false.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static exclusive\_or(*arg0*, *arg1*)
    :   Logical or bitwise exclusive or.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static not\_(*arg*)
    :   Logical or bitwise negation.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static negate(*arg*)
    :   Numerical negation, which negates a uint as a long.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static conditional(*condition*, *if\_true*, *if\_false*)
    :   The second argument if the condition is true, and the third argument otherwise.

        Parameters:
        :   - **condition** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The boolean condition.
            - **if\_true** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value when the condition is true.
            - **if\_false** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value when the condition is false.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the two values have no common type.

    static add(*arg0*, *arg1*)
    :   Numerical addition.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static subtract(*arg0*, *arg1*)
    :   Numerical subtraction.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static multiply(*arg0*, *arg1*)
    :   Numerical multiplication.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static divide(*arg0*, *arg1*)
    :   Numerical division.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static modulo(*arg0*, *arg1*)
    :   Numerical modulo operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Returns:
        :   The remainder of arg0 divided by arg1

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static power(*arg0*, *arg1*)
    :   Numerical power operator.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Returns:
        :   arg0 raised to the power of arg1, at the common type of the operands

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static left\_shift(*arg0*, *arg1*)
    :   Bitwise left shift.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Only the low 5 bits of the count are used for a 32-bit value, and the low 6 bits
        > for a 64-bit value.

    static right\_shift(*arg0*, *arg1*)
    :   Bitwise right shift.

        Parameters:
        :   - **arg0** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first operand.
            - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second operand.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Only the low 5 bits of the count are used for a 32-bit value, and the low 6 bits
        > for a 64-bit value.

    static cast(*arg*, *type*)
    :   A value cast to the given type.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to cast.
            - **type** ([*Type*](#KRPC.Type "KRPC.Type")) – Type to cast the argument to.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A tuple or collection is cast element by element.

    static parameter(*name*, *type*)
    :   A named parameter of a function.

        Parameters:
        :   - **name** (*str*) – The name of the parameter.
            - **type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the parameter.

        Returns:
        :   A named parameter.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A value is bound to it by name with [`Expression.invoke()`](#KRPC.Expression.invoke "KRPC.Expression.invoke"), or by its position in
        > the parameters of a [`Expression.lambda_()`](#KRPC.Expression.lambda_ "KRPC.Expression.lambda_").

    static lambda\_(*parameters*, *body*)
    :   A function with the given parameters and body.

        Parameters:
        :   - **parameters** (*list*) – The parameters of the function.
            - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The body of the function.

        Returns:
        :   A function.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The body is an expression or a block of statements, and a body with no value is
        > evaluated for its effects. A [`Expression.return_()`](#KRPC.Expression.return_ "KRPC.Expression.return_") or [`Expression.return_nothing()`](#KRPC.Expression.return_nothing "KRPC.Expression.return_nothing")
        > statement ends the evaluation.

    static invoke(*function*, *args*)
    :   A function call.

        Parameters:
        :   - **function** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function to call.
            - **args** (*dict*) – The arguments to call the function with.

        Returns:
        :   A function call.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static block(*statements*)
    :   A block of statements, whose value is that of its last statement.

        Parameters:
        :   **statements** (*list*) – The statements.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static block\_with\_variables(*variables*, *statements*)
    :   A block of statements with local variables.

        Parameters:
        :   - **variables** (*list*) – The local variables of the block.
            - **statements** (*list*) – The statements.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The value of the block is that of its last statement. The variables are in scope
        > for the statements of the block, including within nested functions.

    static variable(*name*, *type*)
    :   A local variable, declared by [`Expression.block_with_variables()`](#KRPC.Expression.block_with_variables "KRPC.Expression.block_with_variables").

        Parameters:
        :   - **name** (*str*) – The name of the variable.
            - **type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the variable.

        Returns:
        :   A local variable.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static state\_variable(*name*, *initial\_value*)
    :   A variable that keeps its value between evaluations of a function.

        Parameters:
        :   - **name** (*str*) – The name of the variable.
            - **initial\_value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The initial value of the variable.

        Returns:
        :   A state variable.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The initial value is evaluated once, when the variable is created, and gives the
        > variable its type. The value belongs to the variable, so every function that
        > contains it shares it.

    static assign(*variable*, *value*)
    :   An assignment to a local variable, state variable or function parameter.

        Parameters:
        :   - **variable** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The variable to assign to.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to assign.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The value’s type must be assignable to the variable’s type. A numeric value of a
        > different type is widened, and narrowing it requires a cast.

    static if\_then(*condition*, *body*)
    :   An if statement, with no value.

        Parameters:
        :   - **condition** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The boolean condition.
            - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated when the condition is true.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Use [`Expression.conditional()`](#KRPC.Expression.conditional "KRPC.Expression.conditional") for an if-then-else that produces a value.

    static if\_then\_else(*condition*, *body*, *else\_body*)
    :   An if-else statement, with no value.

        Parameters:
        :   - **condition** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The boolean condition.
            - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated when the condition is true.
            - **else\_body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated when the condition is false.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Use [`Expression.conditional()`](#KRPC.Expression.conditional "KRPC.Expression.conditional") for an if-then-else that produces a value.

    static while\_(*condition*, *body*)
    :   A while loop, which evaluates the body while the condition is true.

        Parameters:
        :   - **condition** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The boolean condition.
            - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated on each iteration.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A [`Expression.break_()`](#KRPC.Expression.break_ "KRPC.Expression.break_") or [`Expression.continue_()`](#KRPC.Expression.continue_ "KRPC.Expression.continue_") in the body applies to this loop.

    static for\_each(*variable*, *collection*, *body*)
    :   A loop that evaluates the body once for each value of a collection.

        Parameters:
        :   - **variable** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The loop variable.
            - **collection** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection to iterate over.
            - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated on each iteration.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The loop variable is a local variable declared by an enclosing
        > [`Expression.block_with_variables()`](#KRPC.Expression.block_with_variables "KRPC.Expression.block_with_variables"), or a parameter of an enclosing [`Expression.lambda_()`](#KRPC.Expression.lambda_ "KRPC.Expression.lambda_").
        > A [`Expression.break_()`](#KRPC.Expression.break_ "KRPC.Expression.break_") or [`Expression.continue_()`](#KRPC.Expression.continue_ "KRPC.Expression.continue_") in the body applies to this loop.

    static break\_()
    :   A break statement, which ends the enclosing loop.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static continue\_()
    :   A continue statement, which skips to the next iteration of the enclosing loop.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static return\_(*value*)
    :   A return statement, which ends the enclosing function with a value.

        Parameters:
        :   **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to return.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static return\_nothing()
    :   A return statement, which ends the enclosing function with no value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static throw(*service*, *name*, *message*)
    :   A throw statement, which raises the named exception.

        Parameters:
        :   - **service** (*str*) – The name of the service the exception is defined in.
            - **name** (*str*) – The name of the exception.
            - **message** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The message of the exception.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The client receives the exception as an error from the function, or from the stream
        > or event evaluating it. Throws an exception if the service does not declare it.

    static try\_catch(*body*, *service*, *name*, *message*, *handler*)
    :   A try-catch statement that handles the named exception.

        Parameters:
        :   - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement to evaluate.
            - **service** (*str*) – The name of the service the exception is defined in.
            - **name** (*str*) – The name of the exception.
            - **message** ([*Expression*](#KRPC.Expression "KRPC.Expression") *or* *None*) – A string variable assigned the exception’s message, or null.
            - **handler** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated when the exception is caught.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Only an exception a service declares can be named, and [`Expression.try_catch_all()`](#KRPC.Expression.try_catch_all "KRPC.Expression.try_catch_all")
        > handles any exception. Throws an exception if the service does not declare it.

    static try\_catch\_all(*body*, *message*, *handler*)
    :   A try-catch statement that handles any exception.

        Parameters:
        :   - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement to evaluate.
            - **message** ([*Expression*](#KRPC.Expression "KRPC.Expression") *or* *None*) – A string variable assigned the exception’s message, or null.
            - **handler** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated when an exception is caught.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static try\_finally(*body*, *finalizer*)
    :   A try-finally statement.

        Parameters:
        :   - **body** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement to evaluate.
            - **finalizer** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The statement evaluated after the body.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The finalizer is evaluated after the body, whether or not the body raises an
        > exception. A break, continue or return cannot leave the finalizer.

    static create\_tuple(*elements*)
    :   A tuple of the given elements. Throws an exception if there are no elements or more
        than seven.

        Parameters:
        :   **elements** (*list*) – The elements.

        Returns:
        :   The tuple.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_struct(*type*, *field\_values*)
    :   A structure with the given field values. Throws an exception if the field values do
        not match its fields.

        Parameters:
        :   - **type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the structure.
            - **field\_values** (*list*) – The values of the structure’s fields, in the order the structure declares them.

        Returns:
        :   The structure.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_list(*values*)
    :   A list of the given values, with numbers widened to their common type. Throws an
        exception if there are no values.

        Parameters:
        :   **values** (*list*) – The values.

        Returns:
        :   The list.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_set(*values*)
    :   A set of the given values, with numbers widened to their common type. Throws an
        exception if there are no values, or a value is null.

        Parameters:
        :   **values** (*set*) – The values.

        Returns:
        :   The set.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The values are evaluated in an unspecified order.

    static create\_dictionary(*keys*, *values*)
    :   A dictionary of the given keys and corresponding values, with numbers widened to
        their common type. Throws an exception if there are no keys, the counts differ, the
        keys are not of a valid key type, or a key is null.

        Parameters:
        :   - **keys** (*list*) – The keys.
            - **values** (*list*) – The values.

        Returns:
        :   The dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A repeated key takes the last of its values.

    static create\_empty\_list(*value\_type*)
    :   An empty list of the given value type.

        Parameters:
        :   **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the values the list holds.

        Returns:
        :   The empty list.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_empty\_set(*value\_type*)
    :   An empty set of the given value type. Throws an exception if the value type is
        nullable.

        Parameters:
        :   **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the values the set holds.

        Returns:
        :   The empty set.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static create\_empty\_dictionary(*key\_type*, *value\_type*)
    :   An empty dictionary of the given key and value types. Throws an exception if the
        key type is nullable or not a valid key type.

        Parameters:
        :   - **key\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the dictionary’s keys.
            - **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the dictionary’s values.

        Returns:
        :   The empty dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static append(*collection*, *value*)
    :   A statement adding a value to a list or a set.

        Parameters:
        :   - **collection** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set to add to.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to add.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The value goes on the end of a list. A set holding the value is left unchanged.

    static set(*collection*, *index*, *value*)
    :   A statement setting an element of a list, or an entry of a dictionary.

        Parameters:
        :   - **collection** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or dictionary.
            - **index** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The index of the element, counting from zero, or the key of the entry.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to set.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A key the dictionary does not hold is added. Throws an exception if the index
        > is out of range for the list.

    static remove(*collection*, *value*)
    :   A removal of a value from a list or a set, or of an entry from a dictionary.

        Parameters:
        :   - **collection** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list, set or dictionary.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to remove, or the key of the entry to remove.

        Returns:
        :   Whether the collection held the value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The first occurrence is removed from a list.

    static remove\_at(*list*, *index*)
    :   A statement removing the element at an index of a list.

        Parameters:
        :   - **list** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list.
            - **index** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The index of the element, counting from zero.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the index is out of range.

    static clear(*collection*)
    :   A statement removing every value from a list or a set, or every entry from a dictionary.

        Parameters:
        :   **collection** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list, set or dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static dictionary\_keys(*dictionary*)
    :   The keys of a dictionary, as a list.

        Parameters:
        :   **dictionary** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The dictionary.

        Returns:
        :   The keys of the dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The collection operations take the keys of a dictionary from here, and its values
        > from [`Expression.dictionary_values()`](#KRPC.Expression.dictionary_values "KRPC.Expression.dictionary_values").

    static dictionary\_values(*dictionary*)
    :   The values of a dictionary, as a list.

        Parameters:
        :   **dictionary** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The dictionary.

        Returns:
        :   The values of the dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static to\_list(*arg*)
    :   A collection as a list.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The collection as a list.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static to\_set(*arg*)
    :   A collection as a set.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The collection as a set.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if a value is null.

    static build\_dictionary(*arg*, *key\_func*, *value\_func*)
    :   A dictionary built from a collection, with a key and a value computed from each
        element.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.
            - **key\_func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function computing an element’s key.
            - **value\_func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function computing an element’s value.

        Returns:
        :   The dictionary.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A repeated key takes the last of its values.

    static get(*arg*, *index*)
    :   An element of a tuple, list or dictionary.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The tuple, list or dictionary.
            - **index** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The index or key of the element.

        Returns:
        :   The element.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A tuple takes a constant integer index, a list a zero indexed integer and a
        > dictionary a key. Throws an exception if the index is out of range or the key is
        > missing, and for a tuple when the expression is created.

    static get\_field(*arg*, *name*)
    :   The value of a field of a structure. Throws an exception if the value is not a
        structure, or has no field of the given name.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The structure.
            - **name** (*str*) – The name of the field to access.

        Returns:
        :   The value of the field.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static convert\_to\_string(*arg*)
    :   The string representation of a value.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to convert.

        Returns:
        :   The string representation of the value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > A value other than a number, boolean, string or enumeration converts as .NET
        > formats it, which for a collection is the name of its type. Throws an exception if
        > the value is null.

    static string\_concat(*args*)
    :   The concatenation of strings.

        Parameters:
        :   **args** (*list*) – The strings to concatenate, in order.

        Returns:
        :   The concatenated string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Use [`Expression.convert_to_string()`](#KRPC.Expression.convert_to_string "KRPC.Expression.convert_to_string") to convert other values to strings.

    static string\_length(*arg*)
    :   The number of characters in a string.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.

        Returns:
        :   The length of the string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_get(*arg*, *index*)
    :   The character at the given position in a string, as a string of length one.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.
            - **index** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The position of the character, counting from zero.

        Returns:
        :   The character at the given position.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the index is out of range.

    static string\_substring(*arg*, *start*, *length*)
    :   Part of a string.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.
            - **start** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The position to start at, counting from zero.
            - **length** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The number of characters to take.

        Returns:
        :   The substring.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the start or the length is out of range.

    static string\_index\_of(*arg*, *value*)
    :   The position of the first occurrence of one string within another.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to search.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to search for.

        Returns:
        :   The position of the value, counting from zero, or -1 if it does not occur.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_contains(*arg*, *value*)
    :   Whether one string occurs within another.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to search.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to search for.

        Returns:
        :   Whether the value occurs in the string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_starts\_with(*arg*, *value*)
    :   Whether a string starts with another.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to test.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to test for.

        Returns:
        :   Whether the string starts with the value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_ends\_with(*arg*, *value*)
    :   Whether a string ends with another.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to test.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to test for.

        Returns:
        :   Whether the string ends with the value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_to\_upper(*arg*)
    :   A string in upper case, independently of the game’s language.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.

        Returns:
        :   The string in upper case.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_to\_lower(*arg*)
    :   A string in lower case, independently of the game’s language.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.

        Returns:
        :   The string in lower case.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_trim(*arg*)
    :   A string with white space removed from both ends.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.

        Returns:
        :   The trimmed string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_trim\_start(*arg*)
    :   A string with white space removed from its start.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.

        Returns:
        :   The trimmed string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_trim\_end(*arg*)
    :   A string with white space removed from its end.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string.

        Returns:
        :   The trimmed string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_replace(*arg*, *old\_value*, *new\_value*)
    :   A string with every occurrence of one string replaced by another.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to search.
            - **old\_value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to replace.
            - **new\_value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to replace it with.

        Returns:
        :   The string with the replacements made.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the string to replace is empty.

    static string\_split(*arg*, *separator*)
    :   The parts of a string separated by another string.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to split.
            - **separator** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string that separates the parts.

        Returns:
        :   The parts of the string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static string\_join(*separator*, *values*)
    :   Strings joined together, separated by another string.

        Parameters:
        :   - **separator** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The string to put between the values.
            - **values** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The strings to join.

        Returns:
        :   The joined string.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static count(*arg*)
    :   The number of values in a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list, set or dictionary.

        Returns:
        :   The number of elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static sum(*arg*)
    :   The sum of the values of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The sum of the elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static max(*arg*)
    :   The largest value of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The largest value in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty and its values are not nullable.

    static min(*arg*)
    :   The smallest value of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The smallest value in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty and its values are not nullable.

    static average(*arg*)
    :   The average of the values of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.

        Returns:
        :   The average of the elements in the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty and its values are not nullable.

    static select(*arg*, *func*)
    :   The result of a function for each value of a collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function, taking an element of the collection.

        Returns:
        :   The collection of function results.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static where(*arg*, *func*)
    :   The values of a collection for which a predicate function returns true.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The predicate function, taking an element of the collection.

        Returns:
        :   The filtered collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static select\_many(*arg*, *func*)
    :   The values of the collections a function returns for each value of a collection,
        flattened into one.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The list or set.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The function, taking an element of the collection and returning a collection.

        Returns:
        :   The flattened collection of function results.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static contains(*arg*, *value*)
    :   Whether a collection contains a value.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **value** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The value to look for.

        Returns:
        :   Whether the collection contains the value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static contains\_key(*dictionary*, *key*)
    :   Whether a dictionary contains a key.

        Parameters:
        :   - **dictionary** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The dictionary.
            - **key** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The key to look for.

        Returns:
        :   Whether the dictionary contains the key.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static aggregate(*arg*, *func*)
    :   The result of applying an accumulator function over a collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The accumulator function.

        Returns:
        :   The accumulated value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty.

    static aggregate\_with\_seed(*arg*, *seed*, *func*)
    :   The result of applying an accumulator function over a collection, starting from a seed.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **seed** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The seed value.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The accumulator function.

        Returns:
        :   The accumulated value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The seed widens to the type of the accumulated value, the function’s result.

    static concat(*arg1*, *arg2*)
    :   The values of one collection followed by those of another.

        Parameters:
        :   - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first collection.
            - **arg2** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second collection.

        Returns:
        :   The first collection followed by the second.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static skip(*arg*, *count*)
    :   A collection without its first count values.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **count** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The number of values to skip.

        Returns:
        :   The collection without its first count values.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static take(*arg*, *count*)
    :   The first count values of a collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **count** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The number of values to take.

        Returns:
        :   The first count values of the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static order\_by(*arg*, *key*)
    :   The values of a collection ordered by the key a function produces for each of them.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection to order.
            - **key** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – A function producing the key to order values by.

        Returns:
        :   The ordered collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static all(*arg*, *predicate*)
    :   Whether a predicate function returns true for every value of a collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **predicate** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The predicate function.

        Returns:
        :   Whether all items satisfy the predicate.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static any(*arg*, *predicate*)
    :   Whether a predicate function returns true for any value of a collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **predicate** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The predicate function.

        Returns:
        :   Whether any item satisfies the predicate.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

    static first(*arg*)
    :   The first value of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The first value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty.

    static last(*arg*)
    :   The last value of a collection.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The last value.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty.

    static element\_at(*arg*, *index*)
    :   The value at the given position in a collection.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **index** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The position of the value, counting from zero.

        Returns:
        :   The value at the given position.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the index is out of range.

    static min\_by(*arg*, *key*)
    :   The value of a collection with the smallest key.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **key** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – A function producing the key to compare values by.

        Returns:
        :   The value with the smallest key.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty.

    static max\_by(*arg*, *key*)
    :   The value of a collection with the largest key.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **key** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – A function producing the key to compare values by.

        Returns:
        :   The value with the largest key.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > Throws an exception if the collection is empty.

    static distinct(*arg*)
    :   The values of a collection with duplicates removed, keeping the first
        occurrence of each.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The distinct values of the collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static reverse(*arg*)
    :   The values of a collection in the opposite order.

        Parameters:
        :   **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.

        Returns:
        :   The reversed collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static union(*arg1*, *arg2*)
    :   The values in either of two collections, with duplicates removed.

        Parameters:
        :   - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first collection.
            - **arg2** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second collection.

        Returns:
        :   The union of the two collections.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static intersect(*arg1*, *arg2*)
    :   The values in both of two collections, with duplicates removed.

        Parameters:
        :   - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first collection.
            - **arg2** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second collection.

        Returns:
        :   The intersection of the two collections.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static except\_(*arg1*, *arg2*)
    :   The values of the first collection that are not in the second, with
        duplicates removed.

        Parameters:
        :   - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first collection.
            - **arg2** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second collection.

        Returns:
        :   The difference between the two collections.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is lazily evaluated. [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static zip(*arg1*, *arg2*, *func*)
    :   The results of a function applied to the values of two collections in pairs.

        Parameters:
        :   - **arg1** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The first collection.
            - **arg2** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The second collection.
            - **func** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – A function combining a value from each collection.

        Returns:
        :   The combined collection.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

        > **Note**
        >
        > The result is as long as the shorter of the two collections. It is lazily
        > evaluated, and [`Expression.to_list()`](#KRPC.Expression.to_list "KRPC.Expression.to_list") makes it concrete.

    static group\_by(*arg*, *key*)
    :   The values of a collection grouped by the key a function produces for each of them.

        Parameters:
        :   - **arg** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – The collection.
            - **key** ([*Expression*](#KRPC.Expression "KRPC.Expression")) – A function producing the key to group values by.

        Returns:
        :   A dictionary of each key to the values that produced it.

        Return type:
        :   [`Expression`](#KRPC.Expression "KRPC.Expression")

class Type
:   The type of a value a procedure takes or returns.

    static double()
    :   A double precision floating point number type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static float()
    :   A single precision floating point number type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static int()
    :   A signed 32-bit integer type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static long()
    :   A signed 64-bit integer type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static u\_int()
    :   An unsigned 32-bit integer type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static u\_long()
    :   An unsigned 64-bit integer type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static bool()
    :   A boolean type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static string()
    :   A string type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static bytes()
    :   A byte sequence type.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static class\_type(*service*, *name*)
    :   The type of a class defined by a service.

        Parameters:
        :   - **service** (*str*) – The name of the service the class is defined in.
            - **name** (*str*) – The name of the class.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static enumeration\_type(*service*, *name*)
    :   The type of an enumeration defined by a service.

        Parameters:
        :   - **service** (*str*) – The name of the service the enumeration is defined in.
            - **name** (*str*) – The name of the enumeration.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static struct\_type(*service*, *name*)
    :   The type of a structure defined by a service.

        Parameters:
        :   - **service** (*str*) – The name of the service the structure is defined in.
            - **name** (*str*) – The name of the structure.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static tuple\_type(*value\_types*)
    :   A tuple type. Throws an exception if there are no element types or more than seven.

        Parameters:
        :   **value\_types** (*list*) – The types of the elements of the tuple.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static list\_type(*value\_type*)
    :   A list type.

        Parameters:
        :   **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the elements of the list.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static set\_type(*value\_type*)
    :   A set type. Throws an exception if the element type is nullable.

        Parameters:
        :   **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the elements of the set.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static dictionary\_type(*key\_type*, *value\_type*)
    :   A dictionary type. Throws an exception if the key type is nullable or not a valid
        key type.

        Parameters:
        :   - **key\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the keys of the dictionary.
            - **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the values of the dictionary.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    static nullable\_type(*value\_type*)
    :   A nullable type, holding a value of the given type or null.

        Parameters:
        :   **value\_type** ([*Type*](#KRPC.Type "KRPC.Type")) – The type of the value.

        Return type:
        :   [`Type`](#KRPC.Type "KRPC.Type")

    code
    :   The kind of the type.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`TypeCode`](#KRPC.TypeCode "KRPC.TypeCode")

    service
    :   The service a class, enumeration or structure type is defined in, or an empty string.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    name
    :   The name of a class, enumeration or structure type, or an empty string.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    types
    :   The types of the elements of a tuple, list, set or dictionary type.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Type`](#KRPC.Type "KRPC.Type"))

        > **Note**
        >
        > The value types of a tuple, the value type of a list or set, or the key and value
        > types of a dictionary. An empty list for any other type.

    nullable
    :   Whether a value of the type may be null.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

        > **Note**
        >
        > Any type can be nullable. A string or class value of a type that is not nullable
        > is never null.

class TypeCode
:   The kind of a type, with the values of the communication protocol’s type codes.

    double
    :   A double precision floating point number.

    float
    :   A single precision floating point number.

    s\_int32
    :   A signed 32-bit integer.

    s\_int64
    :   A signed 64-bit integer.

    u\_int32
    :   An unsigned 32-bit integer.

    u\_int64
    :   An unsigned 64-bit integer.

    bool
    :   A boolean.

    string
    :   A string.

    bytes
    :   A sequence of bytes.

    class
    :   A class defined by a service.

    enumeration
    :   An enumeration defined by a service.

    struct
    :   A structure defined by a service.

    tuple
    :   A tuple, with the types of its values in [`Type.types`](#KRPC.Type.types "KRPC.Type.types").

    list
    :   A list, with the type of its values in [`Type.types`](#KRPC.Type.types "KRPC.Type.types").

    set
    :   A set, with the type of its values in [`Type.types`](#KRPC.Type.types "KRPC.Type.types").

    dictionary
    :   A dictionary, with the types of its keys and values in [`Type.types`](#KRPC.Type.types "KRPC.Type.types").
