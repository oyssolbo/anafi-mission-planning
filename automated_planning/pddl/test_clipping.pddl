(define (domain test_clipping)
  (:requirements 
    :strips 
    :durative-actions 
    :typing 
    :numeric-fluents 
  )

  (:types
    location
    echain
    strut
    dummy
  )


  (:predicates
    (set ?st - strut)
    (release ?st - strut)
    (disconnect ?st - strut)
    (available ?st - strut)
    (unused ?st - strut)
    (following ?st1 - strut ?st2 - strut)
    (ending ?st - strut)
    (started ?ec - echain)
    (notStarted ?ec - echain)
    (performedChain ?ec - echain)
    (setDummy ?d - dummy)
    (notSetDummy ?d - dummy)
  )


  (:functions
    (amountWaiting ?p - port)
    (change ?p - port)
    (limit ?d - dummy)
  )

;define actions here
(:durative-action start_chain
    :parameters (?ec - echain ?first_strut - strut)
    :duration (= ?duration 0.003)
    :condition (and 
        (at start (notStarted ?ec))
        (over all (available ?first_strut))
        (at end (set ?first_strut))
    )
    :effect (and 
        (at start (not (notStarted ?ec)))
        (at start (started ?ec))
        (at end (disconnect ?first_strut))
    )
)


(:durative-action increase_people
    :parameters (?p - port ?st1 - strut ?st2 - strut ?ec - echain)
    :duration (= ?duration 300)
    :condition (and 
        (at start (set ?st1))
        (at start (unused ?st1))
        (over all (started ?ec))
        (over all (following ?st1 ?st2))
        (at end (set ?st2))
    )
    :effect (and 
        (at start (release ?st1))
        (at start (not(unused ?st1)))
        (at end (disconnect ?st2))
        (at end (increase (amountWaiting ?p) (change ?p)))
    )
)

(:durative-action struting
    :parameters (?st - strut ?next_strut - strut ?ec - echain)
    :duration (= ?duration 0.003)
    :condition (and 
        (at start (unused ?st))
        (over all (following ?st ?next_strut))
        (over all (available ?st))
        (over all (started ?ec))
        (at end (release ?st))
        (at end (disconnect ?st))
    )
    :effect (and 
        (at start (set ?st))
        (at end (available ?next_strut))
    )
)

(:durative-action endStrut
    :parameters (?endstrut - strut ?ec - echain)
    :duration (= ?duration 0.003)
    :condition (and 
        (over all (available ?endstrut))
        (over all (started ?ec))
        (over all (ending ?endstrut))
        (at end (disconnect ?endstrut))
    )
    :effect (and 
        (at start (set ?endstrut))
        (at end (performedChain ?ec))
    )
)


(:durative-action dummy_action
    :parameters (?d - dummy ?p - port ?ec - echain)
    :duration (= ?duration 200)
    :condition (and 
        (at start (notSetDummy ?d))
        (over all (started ?ec))
        (at start (>= (amountWaiting ?p) (limit ?d)))
    )
    :effect (and 
        (at start (not(notSetDummy ?d)))
        (at end (setDummy ?d))
    )
)


)