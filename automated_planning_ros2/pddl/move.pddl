(define (domain simple)
  (:requirements 
    :strips 
    :typing 
    :adl 
    :fluents 
    :durative-actions
  )

  ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:types
    drone
    location
  );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:predicates
    (drone_at ?d - drone ?loc - location)
    (path ?loc_from - location ?loc_to - location)
    (landed ?d - drone)
    (not_landed ?d - drone)
  );; end Predicates ;;;;;;;;;;;;;;;;;;;;
  
  ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:functions
  );; end Functions ;;;;;;;;;;;;;;;;;;;;
  
  ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:durative-action move
      :parameters (?d - drone ?loc_from - location ?loc_to - location)
      :duration ( = ?duration 5)
      :condition (and
        (at start(path ?loc_from ?loc_to))
        (at start(drone_at ?d ?loc_from))
      )
      :effect (and
        (at start(not(drone_at ?d ?loc_from)))
        (at end(drone_at ?d ?loc_to))
      )
  )

  (:durative-action land
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration 5)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(not_landed ?d))
      )
      :effect (and
        (at end(and
          (landed ?d)
          (not (not_landed ?d))
        ))
      )
  )

  (:durative-action takeoff
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration 5)
      :condition (and
          (at start(drone_at ?d ?loc))
          (at start(landed ?d))
      )
      :effect (and
          (at end(not(landed ?d)))
          (at end(not_landed ?d))
      )
  )

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
