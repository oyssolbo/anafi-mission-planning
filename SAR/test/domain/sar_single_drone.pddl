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
  )

  (:types
    vehicle location locatable equipment - objects
    drone - vehicle
    helipad searcharea loiterarea - location
    person - locatable
    battery marker - equipment 
  )

  (:predicates 
    (at ?drone - drone ?loc - location)
    (path ?loc-from - location ?loc-to - location)
    (available ?loc - location)
    (is ?person - person ?searcharea - searcharea)
    (has ?drone - drone ?equipment - equipment)
    (landed ?drone)
    (searched ?loc - searcharea)
    (searching ?drone)
    (rescue ?person)
  )


  (:durative-action MOVE
    :parameters (
      ?drone - drone 
      ?loc-from - location
      ?loc-to - location
      ?battery - battery
    )
    :duration (= ?duration 15)
    :condition (and 
      (at start (and
        (at ?drone ?loc-from)
        (available ?loc-to) 
        (path ?loc-from ?loc-to)
        (has ?drone ?battery)
        (not (landed ?drone))
        (not (searching ?drone))
      ))
      (over all (and 
        (available ?loc-to)
        (has ?drone ?battery)
        (not (landed ?drone))
      ))
    )
    :effect (and 
      (at start (and 
        (not (at ?drone ?loc-from))
        (available ?loc-from) ; Technically incorrect to assume available at once, as the drone spends some time to get out of an area
      ))
      (at end (and
        (at ?drone ?loc-to) 
        (not (available ?loc-to))
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
      ?loc - searcharea 
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