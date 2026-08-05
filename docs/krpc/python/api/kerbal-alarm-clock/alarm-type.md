# AlarmType

class AlarmType
:   The type of an alarm.

    raw
    :   An alarm for a specific date/time or a specific period in the future.

    maneuver
    :   An alarm based on the next maneuver node on the current ships flight path.
        This node will be stored and can be restored when you come back to the ship.

    maneuver\_auto
    :   See [`AlarmType.maneuver`](#KerbalAlarmClock.AlarmType.maneuver "KerbalAlarmClock.AlarmType.maneuver").

    apoapsis
    :   An alarm for furthest part of the orbit from the planet.

    periapsis
    :   An alarm for nearest part of the orbit from the planet.

    ascending\_node
    :   Ascending node for the targeted object, or equatorial ascending node.

    descending\_node
    :   Descending node for the targeted object, or equatorial descending node.

    closest
    :   An alarm based on the closest approach of this vessel to the targeted
        vessel, some number of orbits into the future.

    contract
    :   An alarm based on the expiry or deadline of contracts in career modes.

    contract\_auto
    :   See [`AlarmType.contract`](#KerbalAlarmClock.AlarmType.contract "KerbalAlarmClock.AlarmType.contract").

    crew
    :   An alarm that is attached to a crew member.

    distance
    :   An alarm that is triggered when a selected target comes within a chosen distance.

    earth\_time
    :   An alarm based on the time in the “Earth” alternative Universe (aka the Real World).

    launch\_rendevous
    :   An alarm that fires as your landed craft passes under the orbit of your target.

    soi\_change
    :   An alarm manually based on when the next SOI point is on the flight path
        or set to continually monitor the active flight path and add alarms as it
        detects SOI changes.

    soi\_change\_auto
    :   See [`AlarmType.soi_change`](#KerbalAlarmClock.AlarmType.soi_change "KerbalAlarmClock.AlarmType.soi_change").

    transfer
    :   An alarm based on Interplanetary Transfer Phase Angles, i.e. when should
        I launch to planet X? Based on Kosmo Not’s post and used in Olex’s
        Calculator.

    transfer\_modelled
    :   See [`AlarmType.transfer`](#KerbalAlarmClock.AlarmType.transfer "KerbalAlarmClock.AlarmType.transfer").

    science\_lab
    :   An alarm for when a science lab has finished processing data.
