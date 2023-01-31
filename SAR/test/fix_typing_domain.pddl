(define 
  (
    domain fix_typing
  )

  (:requirements 
    :durative-actions 
    :typing 
    ; :conditional-effects 
    :negative-preconditions 
    ; :disjunctive-preconditions
    :equality
  )

  (:types
    drone aircraft - vehicle
    helipad searcharea loiterarea - location
    battery 
  )

  (:predicates 
    (at ?v - vehicle ?loc - location)
    (path ?loc-from - location ?loc-to - location)
    (available ?loc - location)
    ; (is ?person - person ?searcharea - searcharea)
    (has ?V - vehicle ?bat - battery)
    (landed ?V - vehicle)
    (searched ?loc - searcharea)
    (searching ?V - vehicle)
    ; (rescue ?person)
  )


  (:durative-action MOVE
    :parameters (
      ?V - vehicle 
      ?loc-from - location
      ?loc-to - location
      ?battery - battery
    )
    :duration (= ?duration 15)
    :condition (and 
      (at start (and
        (at ?V ?loc-from)
        (available ?loc-to) 
        (path ?loc-from ?loc-to)
        (has ?V ?battery)
        (not (landed ?V))
        (not (searching ?V))
      ))
      (over all (and 
        (available ?loc-to)
        (has ?V ?battery)
        (not (landed ?V))
      ))
    )
    :effect (and 
      (at start (and 
        (not (at ?V ?loc-from))
        (available ?loc-from) ; Technically incorrect to assume available at once, as the drone spends some time to get out of an area
      ))
      (at end (and
        (at ?V ?loc-to) 
        (not (available ?loc-to))
      ))
    )
  )


  (:durative-action TAKEOFF
    :parameters (
      ?V - drone 
      ?battery - battery 
      ?helipad - helipad
    )
    :duration (= ?duration 2)
    :condition (and 
      (at start (and 
        (at ?V ?helipad)
        (landed ?V)
        (has ?V ?battery)
      ))
    )
    :effect (and 
      (at end (and 
        (not (landed ?V))
      ))
    )
  )


  (:durative-action LAND
    :parameters (
      ?V - vehicle 
      ?helipad - helipad 
      ?battery - battery
    )
    :duration (= ?duration 2)
    :condition (and 
      (at start (and 
        (at ?V ?helipad)
        (not (landed ?V))
        (not (searching ?V))
      ))
    )
    :effect (and 
      (at end (and
        (landed ?V) 
        (not (has ?V ?battery)) ; Assuming that the battery becomes empty when landing the drone
      ))
    )
  )


  (:durative-action SEARCH
    :parameters (
      ?loc - searcharea 
      ?battery - battery 
      ?V - drone
      ; ?person - person
      ; ?marker - marker
    )
    :duration (= ?duration 60)
    :condition (and 
      (at start (and 
        ; (available ?loc)
        (has ?V ?battery)
        (not (searched ?loc))
        ; (not (landed ?V))
        (at ?V ?loc)
        ; (not (searching ?V))
        ; (has ?V ?marker)
      ))
      (over all (and 
        (has ?V ?battery) 
        ; (available ?loc) ; In case that another vehicle enters the search area - but this will crash with the drone being available
        (not (searched ?loc)) ; not searched in case new information about which areas are searched are received
      ))
    )
    :effect (and 
      (at start (and 
        (searching ?V)
      ))
      (at end (and
        (searched ?loc)
        (not (searching ?V))
        ; (not (has ?V ?marker))
      ))
    )
  )

  (:durative-action RESUPPLY
    :parameters (
      ?V - drone
      ; ?marker - marker
      ?battery - battery
      ?loc - helipad
    )
    :duration (= ?duration 10)
    :condition (and 
      (at start (and 
        (at ?V ?loc)
        (landed ?V)
      ))
    )
    :effect (and 
      (at end (and 
        ; (has ?V ?marker)
        (has ?V ?battery) 
      ))
    )
  )
  
  

  ; (:action RESUPPLY
  ;   :parameters (
  ;     ?V - drone
  ;     ?marker - marker
  ;     ?battery - battery
  ;     ?helipad - helipad
  ;   )
  ;   :precondition (and 
  ;     (at ?V ?helipad)
  ;     (landed ?V)
  ;     ; (or
  ;     ;   (not (has ?V ?marker))
  ;     ;   (not (has ?V ?battery))
  ;     ; )
  ;   )
  ;   :effect (and
  ;     (has ?V ?marker)
  ;     (has ?V ?battery) 
  ;   )
  ; )
  

)