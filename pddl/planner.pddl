(define (domain simple)
    (:requirements :strips :typing :adl :fluents :durative-actions :universal-preconditions :negative-preconditions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot
        waypoint
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (robot_at ?r - robot ?p - waypoint) ; actual position of robot
        (visited ?r - robot ?wp - waypoint) ; waypoint has been visited by robot
        (inspected ?r - robot ?wp - waypoint) ; waypoint has been inspected by robot
        (lowest_found) ; waypoint is the lowest
        (all_inspected ?r - robot) ; all waypoints have been inspected  
    )
    ;; end Predicates ;;;;;;;;;;;;;;;;;;;;

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:durative-action move
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration (= ?duration 1)
        :condition (and
            (at start (robot_at ?r ?wp1))
            ; (at start (not (robot_at ?r ?wp2)))
        )
        :effect (and
            (at start (not (robot_at ?r ?wp1)))
            (at end (robot_at ?r ?wp2))
            (at end (visited ?r ?wp2))
        )
    )

    (:durative-action inspect
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 1)
        :condition (and
            (at start (visited ?r ?wp))
            (over all (robot_at ?r ?wp))
        )
        :effect (and
            (at end (inspected ?r ?wp))
            ; (at end (increase (waypoints_visited ?r) 1))
            ; (at end (when (>= (waypoints_visited ?r) 4) (all_inspected ?r)))
        )
    )

    (:durative-action find_lower
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 60)
        :condition (and 
            (at start (all_inspected ?r))
        )
        :effect (and 
            (at end (lowest_found))
        )
    )
)
