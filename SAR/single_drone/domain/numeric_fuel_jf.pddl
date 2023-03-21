(
  define 
  (
    domain drone_sar_fuel
  )

  (:requirements 
    :strips 
    :equality 
    :typing 
    :fluents 
    :durative-actions 
    :disjunctive-preconditions 
  )

  (:types
    waypoint 
    robot
  )

  (:predicates
    (robot_at ?v - robot ?wp - waypoint)
    (visited ?wp - waypoint)
    (undocked ?v - robot)
    (docked ?v - robot) 
    (charge_at ?wp - waypoint) ; Charger waypoint
    ; (land_at ?wp - waypoint) ; Location where the drone can land
    (battery_charged ?v - robot)
    (photographed ?wp - waypoint) 
    (home_at ?wp - waypoint) ; Home waypoint
    (home ?v - robot)
  )

  (:functions
    (distance ?wp1 ?wp2 - waypoint) 
    (speed ?v - robot)
    (max_range ?v - robot)
    (state_of_charge ?v - robot)
  )


  ; Move to any waypoint, avoiding terrain
  (:durative-action goto_waypoint
    :parameters (?v - robot ?from ?to - waypoint)
    :duration (= ?duration 10) 
    :condition (and 
      (at start (robot_at ?v ?from))
      (at start (>= (state_of_charge ?v) 60))
      (over all (undocked ?v))
    )
    :effect (and
      (at start (not (robot_at ?v ?from)))
      (decrease (state_of_charge ?v) (* 1 #t))
      ; (at end (decrease (state_of_charge ?v) 30))
      (at end (visited ?to))
      (at end (robot_at ?v ?to))
    )
  )

  ; Docking to home position
  (:durative-action dock_home
    :parameters (?v - robot ?wp - waypoint)
    :duration ( = ?duration 2)
    :condition (and
      (at start (home_at ?wp))
      (over all (robot_at ?v ?wp))
      (at start (undocked ?v))
    )
    :effect (and
      (at end (docked ?v))
      (at start (not (undocked ?v)))
      (at end (home ?v))
    )
  )

  ; Docking to charger
  (:durative-action dock
    :parameters (?v - robot ?wp - waypoint)
    :duration ( = ?duration 1)
    :condition (and
      (at start (charge_at ?wp))
      (over all (robot_at ?v ?wp))
      (at start (undocked ?v))
    )
    :effect (and
      (at end (docked ?v))
      (at start (not (undocked ?v)))
    )
  )

  ; Unocking from charger
  (:durative-action undock
    :parameters (?v - robot ?wp - waypoint)
    :duration ( = ?duration 1)
    :condition (and
      (over all (charge_at ?wp))
      (over all (robot_at ?v ?wp))
      (at start (docked ?v))
    )
    :effect (and
      (at start (not (docked ?v)))
      (at end (undocked ?v))
    )
  ) 

  ; Charging battery. Duration based on battery percentage 
  ;	and a charging speed of 2% capacity per second
  (:durative-action charge
    :parameters (?v - robot ?wp - waypoint)
    :duration ( = ?duration (* 0.5 (- 100 (state_of_charge ?v))))
    :condition (and
      (at start (charge_at ?wp))
      (over all (docked ?v))
      (over all (robot_at ?v ?wp))
      (at start (<= (state_of_charge ?v) 100))
    )
    :effect (and
      (at end (assign (state_of_charge ?v) 100))
    )
  )

  ; Photographing an object of interest
  (:durative-action inspect
    :parameters (?v - robot ?wp - waypoint)
    :duration ( = ?duration 10)
    :condition (and
      (over all (robot_at ?v ?wp))
    )
    :effect (and
      (at end (photographed ?wp))
      (at end (decrease (state_of_charge ?v) 3))
    )
  )
)