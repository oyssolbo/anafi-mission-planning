(define (domain numeric_fuel)

  (:requirements
    :strips
    :typing
    :durative-actions
    :continuous-effects
    :conditional-effects
    :negative-preconditions
    :disjunctive-preconditions
    :equality
    :fluents
    :numeric-fluents
  )

  (:types
    helipad search_area loiter_area emergency_lz - location 
    marker - equipment 
    aircraft ship - vehicle
    drone helicopter - aircraft
    revolt - ship  
    person - rescuable 
    action - task
  )

  (:predicates
    (is_not_resupplying ?drone - drone)
    (at ?drone - drone ?loc - location)
    (path ?loc-from - location ?loc-to - location)
    (available ?loc - location)
    (is_person ?person - person ?search_area - search_area)
    (has ?drone - drone ?equipment - equipment)
    ;(is_landable ?loc - location)
    (has_landed ?drone - drone)
    (has_not_landed ?drone - drone)
    (searched ?loc - location)
    (not_searched ?loc - location)
    (is_searching ?drone - drone)
    (is_not_searching ?drone - drone)
    (is_not_moving ?drone - drone)
    ; (rescue ?person - person)
  )

  (:functions
    (distance ?loc_from - location ?loc_to - location)
    (velocity ?veh - vehicle)      ; This assumes that the drone is in a travel-mode
    (battery_level ?drone - drone)    
  )


  (:durative-action MOVE
    :parameters (
      ?drone - drone 
      ?loc_from - location
      ?loc_to - location
    )
    :duration (= ?duration (/ (distance ?loc_from ?loc_to) (velocity ?drone)))
    :condition (and 
      (at start (and
        (at ?drone ?loc_from)
        (available ?loc_to) 
        (path ?loc_from ?loc_to)
        (has_not_landed ?drone) ; Suddenly the popf does not support the "not" keyword for some fuckings reason... It just segfaults
        (>= (battery_level ?drone) 10)
      ))
      (over all (and
        (available ?loc_to)
        (is_not_searching ?drone)
        ; (has_not_landed ?drone)
        (not (is_not_moving ?drone))
      ))
    )
    :effect (and 
      (at end (and
        (available ?loc_from)
        (not (available ?loc_to))
        (not (at ?drone ?loc_from))
        (at ?drone ?loc_to) 
        (decrease (battery_level ?drone) 10) ; Change this to vary with respect to the fuel usage per time unit / distance
      ))
    )
  )


  (:durative-action TAKEOFF
    :parameters (
      ?drone - drone
      ?helipad - helipad
    )
    :duration (= ?duration 2)
    :condition (and
      (at start (and
        (at ?drone ?helipad)
        (has_landed ?drone)
        (>= (battery_level ?drone) 31)
      ))
      (over all (and 
        (is_not_resupplying ?drone)
        (is_not_moving ?drone)
      ))
    )
    :effect (and
      (at end (and
        ; (decrease (battery_level ?drone) (battery_usage_per_time_unit ?takeoff))
        (not (has_landed ?drone))
        (has_not_landed ?drone)
        (decrease (battery_level ?drone) 1)
      ))
    )
  )


  (:durative-action LAND
    :parameters (
      ?drone - drone
      ?helipad - helipad ; - helipad ; Must appearantly specify the superclass... FML! 
    )
    :duration (= ?duration 2)
    :condition (and
      (at start (and
        (at ?drone ?helipad) ; Close enough to the helipad to count as on it
        (has_not_landed ?drone)
      ))
      (over all (and 
        (is_not_moving ?drone)
      ))
    )
    :effect (and
      (at end (and
        (has_landed ?drone)
        (not (has_not_landed ?drone))
        (decrease (battery_level ?drone) 2) ; duration
      ))
    )
  )


  (:durative-action SEARCH
    :parameters (
      ?loc - location
      ?drone - drone
      ; ?marker - marker
    )
    :duration (= ?duration 60) ; Must determine a duration for a search 
    :condition (and
      (at start (and
        (not_searched ?loc)
        (at ?drone ?loc)
        ; (has ?drone ?marker)
      ))
      (over all (and
        ; (has ?drone ?marker)
        (at ?drone ?loc)
        ; (available ?loc) ; In case that another vehicle enters the search area - but this will crash with the drone being available
        (not_searched ?loc) ; not searched in case new information about which areas are searched are received
      ))
    )
    :effect (and
      (at start (and
        (is_searching ?drone)
        (not (is_not_searching ?drone))
      ))
      (at end (and
        (searched ?loc)
        (not (not_searched ?loc))
        (not (is_searching ?drone))
        (is_not_searching ?drone)
        ; (not (has ?drone ?marker))
        ; (when
        ;   ; Condition 
        ;   (and (is_person ?person ?loc)) 
        ;   ; Effect
        ;   (and ((has ?drone ?marker)))
        ; )
      ))
    )
  )


  (:durative-action RESUPPLY
    :parameters (
      ?drone - drone
      ?marker - marker
      ?loc - helipad
    )
    :duration (= ?duration 10)
    :condition (and
      (at start (and
        (at ?drone ?loc)
        (has_landed ?drone)
      ))
    )
    :effect (and
      (at start (and
        (not (is_not_resupplying ?drone))
      ))
      (at end (and
        (has ?drone ?marker)
        (is_not_resupplying ?drone)
      ))
    )
  )


  (:durative-action CHARGE
    :parameters (
      ?drone - drone
      ?loc - helipad
    )
    :duration ( = ?duration (* 0.1 (- 100 (battery_level ?drone)))) ; Charges 10 percent per time unit
    :condition (and
      (at start (and
        (at ?drone ?loc)
        (has_landed ?drone)
        (< (battery_level ?drone) 100)
      ))
    )
    :effect (and
      (at start (and
        (not (is_not_resupplying ?drone))
      ))
      (at end (and
        (assign (battery_level ?drone) 100)
        (is_not_resupplying ?drone)
      ))
    )
  )
)
