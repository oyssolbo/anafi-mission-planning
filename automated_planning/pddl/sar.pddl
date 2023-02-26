(define (domain search_and_rescue)
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
    person
    marker
    lifevest
  );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:predicates
    (drone_at ?d - drone ?loc - location)
    (person_at ?p - person ?loc - location)
    (path ?loc_from - location ?loc_to - location)
    (landed ?d - drone)
    (searched ?loc - location)
    (not_landed ?d - drone)
    (not_searching ?d - drone)
    (not_rescuing ?d - drone)
    (rescued ?p - person ?loc - location)
  );; end Predicates ;;;;;;;;;;;;;;;;;;;;
  
  ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:functions
    (distance ?loc_from - location ?loc_to - location)
    (move_velocity ?d - drone)
    (search_velocity ?d - drone) 
    (search_distance ?loc - location)
    (battery_charge ?d - drone)
    (num_markers ?d - drone)
    (num_lifevests ?d - drone)    
  );; end Functions ;;;;;;;;;;;;;;;;;;;;
  
  ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:durative-action move
      :parameters (?d - drone ?loc_from - location ?loc_to - location)
      :duration ( = ?duration (/ (distance ?loc_from ?loc_to) (move_velocity ?d)))
      :condition (and
        (at start(path ?loc_from ?loc_to))
        (at start(drone_at ?d ?loc_from))
        (over all(not_landed ?d))
        (over all(not_searching ?d))
        (over all(not_rescuing ?d))
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
      :duration ( = ?duration 2)
      :condition (and
          (at start(drone_at ?d ?loc))
          (at start(landed ?d))
      )
      :effect (and
          (at end(not(landed ?d)))
          (at end(not_landed ?d))
      )
  )

  (:durative-action search
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration (/ (search_distance ?loc) (search_velocity ?d)))
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(not_searching ?d))
        (over all(not_landed ?d))
      )
      :effect (and
        ; (over all(not(not_searching ?d))) ; Why tf does this not work?? Fuck PDDL
        (at start(not(not_searching ?d)))
        (at end(not_searching ?d))
      )
  )

  (:durative-action rescue
      :parameters (?d - drone ?loc - location ?p - person)
      :duration ( = ?duration 10)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(person_at ?loc))
        (over all(not_landed ?d))
      )
      :effect (and
        ; (over all(not(not_searching ?d))) ; Why tf does this not work?? Fuck PDDL
        (at start(not(not_rescuing ?d)))
        (at end(not_rescuing ?d))
        (at end(rescued ?p ?loc))
        (at end(not(person_at ?p ?loc))) ; Assuming that the person is assumed known from before
      )
  )


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
