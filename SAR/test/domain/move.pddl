(define 
  (
    domain move_domain
  )

  (:requirements 
    :durative-actions 
    :typing 
    :conditional-effects 
    :negative-preconditions 
    :equality
  )

  (:types
    vehicle location - objects
    drone - vehicle
  )

  (:predicates 
    (at ?drone - drone ?loc - location)
    (path ?loc-from - location ?loc-to - location)
    (available ?loc - location)
  )


  (:durative-action MOVE
    :parameters (
      ?drone - drone 
      ?loc-from - location
      ?loc-to - location
    )
    :duration (= ?duration 1)
    :condition (and 
      (at start (and
        (at ?drone ?loc-from)
        (available ?loc-to) 
        (path ?loc-from ?loc-to)
      ))
    )
    :effect (and 
      (at end (and
        (available ?loc-from)
        (at ?drone ?loc-to) 
      ))
    )
  )

)