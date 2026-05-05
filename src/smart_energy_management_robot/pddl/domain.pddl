(define (domain energy_management)
  (:requirements :strips :typing)

  (:types waypoint)

  (:predicates
    (robot_at ?wp - waypoint)
    (visited ?wp - waypoint)
    (connected ?from ?to - waypoint)
    (critical_energy_active)
    (high_energy_active)
    (is_critical_wp ?wp - waypoint)
    (is_high_wp ?wp - waypoint)
  )

  (:action visit_critical
    :parameters (?from ?to - waypoint)
    :precondition (and
      (robot_at ?from)
      (connected ?from ?to)
      (is_critical_wp ?to)
      (critical_energy_active)
    )
    :effect (and
      (robot_at ?to)
      (not (robot_at ?from))
      (visited ?to)
      (not (critical_energy_active))
    )
  )

  (:action visit_high
    :parameters (?from ?to - waypoint)
    :precondition (and
      (robot_at ?from)
      (connected ?from ?to)
      (is_high_wp ?to)
      (high_energy_active)
      (not (critical_energy_active))
    )
    :effect (and
      (robot_at ?to)
      (not (robot_at ?from))
      (visited ?to)
      (not (high_energy_active))
    )
  )

  (:action visit_waypoint
    :parameters (?from ?to - waypoint)
    :precondition (and
      (robot_at ?from)
      (connected ?from ?to)
      (not (visited ?to))
      (not (critical_energy_active))
      (not (high_energy_active))
    )
    :effect (and
      (robot_at ?to)
      (not (robot_at ?from))
      (visited ?to)
    )
  )
)
