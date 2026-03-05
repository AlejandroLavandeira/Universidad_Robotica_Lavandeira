(define (domain domain2)
  (:requirements :strips :typing :equality) ; Añadimos :equality por el (not (= ...))

  (:types
      robot waypoint orientation - object
  )

  (:predicates
      (at ?r - robot ?wp - waypoint)
      (robot_has_orientation ?r - robot ?o - orientation)
      (connected ?from - waypoint ?to - waypoint ?o - orientation)
  )

  (:action move
      :parameters (?r - robot ?from ?to - waypoint ?o - orientation)
      :precondition (and
          (at ?r ?from)
          (robot_has_orientation ?r ?o)
          (connected ?from ?to ?o)
      )
      :effect (and
          (not (at ?r ?from))
          (at ?r ?to)
      )
  )

  ; --- NUEVA ACCIÓN AÑADIDA ---
  (:action turn
      :parameters (?r - robot ?from_o ?to_o - orientation)
      :precondition (and
          (robot_has_orientation ?r ?from_o)
          (not (= ?from_o ?to_o)) ; Opcional pero recomendado
      )
      :effect (and
          (not (robot_has_orientation ?r ?from_o))
          (robot_has_orientation ?r ?to_o)
      )
  )
)
