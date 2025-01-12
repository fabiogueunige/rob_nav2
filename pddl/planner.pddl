(define (domain simple)
    (:requirements :strips :typing :adl :fluents :durative-actions)

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
        (reach_lower ?r - robot) ; robot has reached the lower waypoint
        (all_inspected ?r - robot) ; all waypoints have been inspected
    );; end Predicates ;;;;;;;;;;;;;;;;;;;;

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:durative-action move
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration (= ?duration 60)
        :condition (and
            (at start (robot_at ?r ?wp1))
            (at start (not (robot_at ?r ?wp2)))
            (at start (not (visited ?r ?wp2)))
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
            (at start (not (inspected ?r ?wp)))
        )
        :effect (and
            (at end (inspected ?r ?wp))
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

    ; move to lowest !!
    
    ;; Additional helper action to set all_inspected ;;;;;;;
    (:durative-action check_all_inspected
        :parameters (?r - robot)
        :duration (= ?duration 1)
        :condition (and 
            (forall (?wp - waypoint) (inspected ?r ?wp))
        )
        :effect (and 
            (at end (all_inspected ?r))
        )
    )
)
;; Guarda e modifica su git