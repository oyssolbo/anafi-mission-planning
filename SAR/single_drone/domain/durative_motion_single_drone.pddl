(define 
  (
    domain move_domain
  )

  (:requirements 
    :durative-actions 
    :typing 
    :conditional-effects 
    :negative-preconditions 
    :disjunctive-preconditions
    :equality
    :fluents
    ; :preferences ; not supported by parser nor (S)TP
    ; :constraints ; not supported by parser nor (S)TP
  )

  (:types    
    helipad search_area loiter_area - location
    person - locatable
    battery marker - equipment 
    task
    drone 
    location 
    locatable 
  )

  (:predicates 
    (at ?drone - drone ?loc - location)
    (path ?loc_from - location ?loc_to - location)
    (available ?loc - location)
    (is ?person - person ?search_area - search_area)
    (has ?drone - drone ?equipment - equipment)
    (landed ?drone)
    (searched ?loc - search_area)
    (searching ?drone)
    (rescue ?person)
  )

  (:functions
    (distance ?loc_from - location ?loc_to - location)
    (velocity ?drone - drone)      ; This assumes that the drone is in a travel-mode 
    (battery_level ?drone - drone)    ; Only considering the fuel level of the drone here
    (power_req_for_task ?task - task) ; Could use different power levels for each of the tasks
  )


  (:durative-action MOVE
    :parameters (
      ?drone - drone
      ?loc_from - location
      ?loc_to - location
      ?battery - battery
    )
    :duration (= ?duration (/ (distance ?loc_from ?loc_to) (velocity ?drone)))
    :condition (and 
      (at start (and
        (at ?drone ?loc_from)
        (available ?loc_to) 
        (path ?loc_from ?loc_to)
        (has ?drone ?battery)
        (not (landed ?drone))
        (not (searching ?drone))
        ; Would like to have the drone not try to move when the battery level is decreasing, unless it is trying to get back home
        ; Might have to be necessary using preferences to express soft goals. For example it is 
        ; preferred to have searched the areas before battery runs out, however it is strictly speaking
        ; also necessary to ensure the drone is not lost. The tuning of this should be done with domain-experts etc.    
      ))
      (over all (and 
        (available ?loc_to)
        (has ?drone ?battery)
        (not (landed ?drone))
      ))
    )
    :effect (and 
      (at start (and 
        (not (at ?drone ?loc_from))
        (available ?loc_from) ; Technically incorrect to assume available at once, as the drone spends some time to get out of an area
      ))
      (at end (and
        (at ?drone ?loc_to) 
        (not (available ?loc_to))
      ))
    )
  )


  (:durative-action TAKEOFF
    :parameters (
      ?drone - drone 
      ?battery - battery 
      ?helipad - helipad
    )
    :duration (= ?duration 2)
    :condition (and 
      (at start (and 
        (at ?drone ?helipad)
        (landed ?drone)
        (has ?drone ?battery)
      ))
    )
    :effect (and 
      (at end (and 
        (not (landed ?drone))
      ))
    )
  )


  (:durative-action LAND
    :parameters (
      ?drone - drone 
      ?helipad - helipad 
      ?battery - battery
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
        (not (has ?drone ?battery)) ; Assuming that the battery becomes empty when landing the drone
      ))
    )
  )


  (:durative-action SEARCH
    :parameters (
      ?loc - search_area 
      ?battery - battery 
      ?drone - drone
      ; ?person - person
      ?marker - marker
    )
    :duration (= ?duration 60)
    :condition (and 
      (at start (and 
        ; (available ?loc)
        (has ?drone ?battery)
        (not (searched ?loc))
        ; (not (landed ?drone))
        (at ?drone ?loc)
        ; (not (searching ?drone))
        (has ?drone ?marker)
      ))
      (over all (and 
        (has ?drone ?battery) 
        ; (available ?loc) ; In case that another vehicle enters the search area - but this will crash with the drone being available
        (not (searched ?loc)) ; not searched in case new information about which areas are searched are received
      ))
    )
    :effect (and 
      (at start (and 
        (searching ?drone)
      ))
      (at end (and
        (searched ?loc)
        (not (searching ?drone))
        (not (has ?drone ?marker))
      ))
    )
  )

  (:durative-action RESUPPLY
    :parameters (
      ?drone - drone
      ?marker - marker
      ?battery - battery
      ?loc - helipad
    )
    :duration (= ?duration 10)
    :condition (and 
      (at start (and 
        (at ?drone ?loc)
        (landed ?drone)
      ))
    )
    :effect (and 
      (at end (and 
        (has ?drone ?marker)
        (has ?drone ?battery) 
      ))
    )
  )
  
  

  ; (:action RESUPPLY
  ;   :parameters (
  ;     ?drone - drone
  ;     ?marker - marker
  ;     ?battery - battery
  ;     ?helipad - helipad
  ;   )
  ;   :precondition (and 
  ;     (at ?drone ?helipad)
  ;     (landed ?drone)
  ;     ; (or
  ;     ;   (not (has ?drone ?marker))
  ;     ;   (not (has ?drone ?battery))
  ;     ; )
  ;   )
  ;   :effect (and
  ;     (has ?drone ?marker)
  ;     (has ?drone ?battery) 
  ;   )
  ; )
  

)