# Contracts

class ContractManager
:   Contracts manager.
    Obtained by calling [`contract_manager`](./space-center.md#SpaceCenter.contract_manager "SpaceCenter.contract_manager").

    types
    :   A list of all contract types.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   set(str)

    all\_contracts
    :   A list of all contracts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Contract`](#SpaceCenter.Contract "SpaceCenter.Contract"))

    active\_contracts
    :   A list of all active contracts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Contract`](#SpaceCenter.Contract "SpaceCenter.Contract"))

    offered\_contracts
    :   A list of all offered, but unaccepted, contracts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Contract`](#SpaceCenter.Contract "SpaceCenter.Contract"))

    completed\_contracts
    :   A list of all completed contracts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Contract`](#SpaceCenter.Contract "SpaceCenter.Contract"))

    failed\_contracts
    :   A list of all failed contracts.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`Contract`](#SpaceCenter.Contract "SpaceCenter.Contract"))

class Contract
:   A contract. Can be accessed using [`contract_manager`](./space-center.md#SpaceCenter.contract_manager "SpaceCenter.contract_manager").

    type
    :   Type of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    title
    :   Title of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    description
    :   Description of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    notes
    :   Notes for the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    synopsis
    :   Synopsis for the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    keywords
    :   Keywords for the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(str)

    state
    :   State of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`ContractState`](#SpaceCenter.ContractState "SpaceCenter.ContractState")

    seen
    :   Whether the contract has been seen.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    read
    :   Whether the contract has been read.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    active
    :   Whether the contract is active.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    failed
    :   Whether the contract has been failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    can\_be\_canceled
    :   Whether the contract can be canceled.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    can\_be\_declined
    :   Whether the contract can be declined.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    can\_be\_failed
    :   Whether the contract can be failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    accept()
    :   Accept an offered contract.

    cancel()
    :   Cancel an active contract.

    decline()
    :   Decline an offered contract.

    funds\_advance
    :   Funds received when accepting the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    funds\_completion
    :   Funds received on completion of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    funds\_failure
    :   Funds lost if the contract is failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    reputation\_completion
    :   Reputation gained on completion of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    reputation\_failure
    :   Reputation lost if the contract is failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    science\_completion
    :   Science gained on completion of the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    parameters
    :   Parameters for the contract.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ContractParameter`](#SpaceCenter.ContractParameter "SpaceCenter.ContractParameter"))

    date\_accepted
    :   Universal time at which the contract was accepted, in seconds.
        Zero if the contract has not been accepted.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    date\_deadline
    :   Universal time by which the contract must be completed, in seconds.
        Zero if the contract has no deadline.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    date\_expire
    :   Universal time at which the contract offer expires, in seconds.
        Zero if the contract has no expiry date.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    date\_finished
    :   Universal time at which the contract was completed or failed, in seconds.
        Zero if the contract has not finished or KSP did not record a finish time.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

class ContractState
:   The state of a contract. See [`Contract.state`](#SpaceCenter.Contract.state "SpaceCenter.Contract.state").

    active
    :   The contract is active.

    canceled
    :   The contract has been canceled.

    completed
    :   The contract has been completed.

    deadline\_expired
    :   The deadline for the contract has expired.

    declined
    :   The contract has been declined.

    failed
    :   The contract has been failed.

    generated
    :   The contract has been generated.

    offered
    :   The contract has been offered to the player.

    offer\_expired
    :   The contract was offered to the player, but the offer expired.

    withdrawn
    :   The contract has been withdrawn.

class ContractParameter
:   A contract parameter. See [`Contract.parameters`](#SpaceCenter.Contract.parameters "SpaceCenter.Contract.parameters").

    title
    :   Title of the parameter.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    notes
    :   Notes for the parameter.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   str

    children
    :   Child contract parameters.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list([`ContractParameter`](#SpaceCenter.ContractParameter "SpaceCenter.ContractParameter"))

    completed
    :   Whether the parameter has been completed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    failed
    :   Whether the parameter has been failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    optional
    :   Whether the contract parameter is optional.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   bool

    funds\_completion
    :   Funds received on completion of the contract parameter.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    funds\_failure
    :   Funds lost if the contract parameter is failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    reputation\_completion
    :   Reputation gained on completion of the contract parameter.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    reputation\_failure
    :   Reputation lost if the contract parameter is failed.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float

    science\_completion
    :   Science gained on completion of the contract parameter.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   float
