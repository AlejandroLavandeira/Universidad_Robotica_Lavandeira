(define (domain domain1)

  (:requirements :strips :typing)

(:types
    robot waypoint orientation - object
)

(:predicates
    (at ?r - robot ?wp - waypoint)
    (robot_has_orientation ?r - robot ?o - orientation)
    (connected ?from - waypoint ?to - waypoint ?o - orientation)
)


(:action move
    :parameters (?r - robot ?from ?to - waypoint ?o - orientation)
    :precondition (and
        (at ?r ?from)
        (robot_has_orientation ?r ?o)
        (connected ?from ?to ?o)
    )
    :effect (and
        (not (at ?r ?from))
        (at ?r ?to)
    )
)
)
