(define (domain numeric_fuel)

  (:requirements
    :strips
    :typing
    :durative-actions
    :conditional-effects
    :negative-preconditions
    :disjunctive-preconditions
    :equality
    :fluents
    :numeric-fluents
  )

  (:types
    drone - vehicle
    helipad search_area loiter_area emergency_lz - location 
    marker - equipment

    ; helipad search_area loiter_area - location
    ; person - locatable
    ; marker - equipment
    ; aircraft ship - vehicle
    ; drone helicopter - aircraft
    ; action
  )

  (:predicates
    (at ?vehicle - vehicle ?loc - location)
    (path ?loc_from - location ?loc_to - location)
    (available ?loc - location)
    ; (is ?person - person ?search_area - search_area)
    (has ?drone - drone ?equipment - equipment)
    (is_landable ?loc - location)
    (landed ?drone - drone)
    (searched ?loc - search_area)
    (searching ?drone - drone)
    ; (rescue ?person - person)
  )

  (:functions
    (distance ?loc_from - location ?loc_to - location)
    (velocity ?drone - drone)      ; This assumes that the drone is in a travel-mode
    (battery_level ?drone - drone)    ; Only considering the fuel level of the drone here
    ; (min_power_lvl_to_start_action ?action - action) ; Could use different power levels for each of the tasks
    ; (battery_usage_per_time_unit ?action - action)
    ; (time ?action)                      ; Some tasks may have different timing requirements
  )


  (:durative-action MOVE
    :parameters (
      ?drone - drone
      ?loc_from - location
      ?loc_to - location
    )
    :duration (= ?duration 1);(/ (distance ?loc_from ?loc_to) (velocity ?drone)))
    :condition (and
      (at start (and
        ; Need to find a way to constrain with respect to the current fuel level
        (at ?drone ?loc_from)
        (available ?loc_to)
        (path ?loc_from ?loc_to)
        (not (landed ?drone))
        (not (searching ?drone))
      ))
      (over all (and
        (available ?loc_to)
        (not (landed ?drone))
      ))
    )
    :effect (and
      (at start (and
        (not (at ?drone ?loc_from))
        (available ?loc_from) ; Technically incorrect to assume available at once, as the drone spends some time to get out of an area
      ))
      (at end (and
        ; (decrease (battery_level ?drone) 10) ; hardcoded value for now
        (at ?drone ?loc_to)
        (not (available ?loc_to))
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
      
        ; (>= (battery_level ?drone) 10) ; Why tf does this not work???
        (at ?drone ?helipad)
        (landed ?drone)
        ; (>= (battery_level ?drone) (min_power_lvl_to_start_action ?takeoff))
        ; (has ?drone ?battery)
      ))
    )
    :effect (and
      (at end (and
        ; (decrease (battery_level ?drone) (battery_usage_per_time_unit ?takeoff))
        (not (landed ?drone))
        ; (decrease (battery_level ?drone) 1)
      ))
    )
  )


  (:durative-action LAND
    :parameters (
      ?drone - drone
      ?helipad - helipad
      ; ?battery - battery
    )
    :duration (= ?duration 2)
    :condition (and
      (at start (and
        (at ?drone ?helipad)
        (not (landed ?drone))
        (not (searching ?drone))
      ))
    )
    :effect (and
      (at end (and
        (landed ?drone)
        ; (not (has ?drone ?battery)) ; Assuming that the battery becomes empty when landing the drone
      ))
    )
  )


  ; (:durative-action SEARCH
  ;   :parameters (
  ;     ?loc - search_area
  ;     ?battery - battery
  ;     ?drone - drone
  ;     ; ?person - person
  ;     ?marker - marker
  ;   )
  ;   :duration (= ?duration 60)
  ;   :condition (and
  ;     (at start (and
  ;       ; (available ?loc)
  ;       (has ?drone ?battery)
  ;       (not (searched ?loc))
  ;       ; (not (landed ?drone))
  ;       (at ?drone ?loc)
  ;       ; (not (searching ?drone))
  ;       (has ?drone ?marker)
  ;     ))
  ;     (over all (and
  ;       (has ?drone ?battery)
  ;       ; (available ?loc) ; In case that another vehicle enters the search area - but this will crash with the drone being available
  ;       (not (searched ?loc)) ; not searched in case new information about which areas are searched are received
  ;     ))
  ;   )
  ;   :effect (and
  ;     (at start (and
  ;       (searching ?drone)
  ;     ))
  ;     (at end (and
  ;       (searched ?loc)
  ;       (not (searching ?drone))
  ;       (not (has ?drone ?marker))
  ;     ))
  ;   )
  ; )

  ; (:durative-action RESUPPLY
  ;   :parameters (
  ;     ?drone - drone
  ;     ?marker - marker
  ;     ?battery - battery
  ;     ?loc - helipad
  ;   )
  ;   :duration (= ?duration 10)
  ;   :condition (and
  ;     (at start (and
  ;       (at ?drone ?loc)
  ;       (landed ?drone)
  ;     ))
  ;   )
  ;   :effect (and
  ;     (at end (and
  ;       (has ?drone ?marker)
  ;       (has ?drone ?battery)
  ;     ))
  ;   )
  ; )


  ; (:durative-action CHARGE
  ;   :parameters (
  ;     ?drone - drone
  ;     ?battery - battery
  ;     ?loc - helipad
  ;   )
  ;   :duration ( = ?duration (* 0.5 (- 100 (battery_level ?drone)))) ; Charges 2 percent per time unit
  ;   :condition (and
  ;     (at start (and
  ;       (at ?drone ?loc)
  ;       (landed ?drone)
  ;       (< (battery_level ?drone) 100)
  ;     ))
  ;   )
  ;   :effect (and
  ;     (at end (and
  ;       (assign (battery_level ?drone) 100)
  ;     ))
  ;   )
  ; )



)