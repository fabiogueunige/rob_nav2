(define (domain rob_nav2)
  (:requirements :durative-actions :typing)
  (:types robot waypoint)

  (:predicates
    (robot_at ?r - robot ?wp - waypoint)
    (visited ?r - robot ?wp - waypoint)
    (inspected ?r - robot ?wp - waypoint)
    (all_inspected ?r - robot)
    (lowest_found ?r - robot)
  )

  (:durative-action move
    :parameters (?r - robot ?wp1 - waypoint ?wp2 - waypoint)
    :duration (= ?duration 1)
    :condition (and
      (at start (robot_at ?r ?wp1))
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
      (at start (robot_at ?r ?wp))
      (at start (visited ?r ?wp))
    )
    :effect (and
      (at end (inspected ?r ?wp))
    )
  )

  (:durative-action find_lowest
    :parameters (?r - robot)
    :duration (= ?duration 1)
    :condition (and 
      (at start (all_inspected ?r))
    )
    :effect (and 
      (at end (lowest_found ?r))
    )
  )
)