(define (problem problem4)
  (:domain domain4)

  (:objects
    kenny - robot
    box1 box2 - box
    wp00 wp01 wp02 wp03
    wp10 wp11 wp12 wp13
    wp20 wp21 wp22 wp23
    wp30 wp31 wp32 wp33 - waypoint
    up down left right - orientation
  )

  (:init

    ; -- Estado inicial del Robot
    (at kenny wp00)
    (robot_has_orientation kenny down)

    ; -- Posición inicial de las Cajas
    (box_at box1 wp11)
    (box_at box2 wp12)

    ; -- Celdas que están vacías al inicio
    (is_empty wp01) (is_empty wp02) (is_empty wp03)
    (is_empty wp10)                 (is_empty wp13)
    (is_empty wp20) (is_empty wp21) (is_empty wp22) (is_empty wp23)
    (is_empty wp30) (is_empty wp31) (is_empty wp32) (is_empty wp33)

    ; -- Topología del mapa (Conexiones)
    ; Conexiones Verticales
    (connected wp00 wp10 down) (connected wp10 wp00 up)
    (connected wp10 wp20 down) (connected wp20 wp10 up)
    (connected wp20 wp30 down) (connected wp30 wp20 up)
    (connected wp01 wp11 down) (connected wp11 wp01 up)
    (connected wp11 wp21 down) (connected wp21 wp11 up)
    (connected wp21 wp31 down) (connected wp31 wp21 up)
    (connected wp02 wp12 down) (connected wp12 wp02 up)
    (connected wp12 wp22 down) (connected wp22 wp12 up)
    (connected wp22 wp32 down) (connected wp32 wp22 up)
    (connected wp03 wp13 down) (connected wp13 wp03 up)
    (connected wp13 wp23 down) (connected wp23 wp13 up)
    (connected wp23 wp33 down) (connected wp33 wp23 up)
    ; Conexiones Horizontales
    (connected wp00 wp01 right) (connected wp01 wp00 left)
    (connected wp01 wp02 right) (connected wp02 wp01 left)
    (connected wp02 wp03 right) (connected wp03 wp02 left)
    (connected wp10 wp11 right) (connected wp11 wp10 left)
    (connected wp11 wp12 right) (connected wp12 wp11 left)
    (connected wp12 wp13 right) (connected wp13 wp12 left)
    (connected wp20 wp21 right) (connected wp21 wp20 left)
    (connected wp21 wp22 right) (connected wp22 wp21 left)
    (connected wp22 wp23 right) (connected wp23 wp22 left)
    (connected wp30 wp31 right) (connected wp31 wp30 left)
    (connected wp31 wp32 right) (connected wp32 wp31 left)
    (connected wp32 wp33 right) (connected wp33 wp32 left)


    ;; ----------------------------------------------------
    ;; -- ESTADO DE FLUENTS (VALORES NUMÉRICOS)
    ;; ----------------------------------------------------

    ; -- Batería inicial y coste de empuje
    (= (battery-level kenny) 100)
    (= (push_cost) 20)
    (= (turn_cost) 1)

    ; -- Costes de movimiento (Verticales=10, Horizontales=5)
    ; Costes Verticales (más caros)
    (= (move_cost wp00 wp10) 10) (= (move_cost wp10 wp00) 10)
    (= (move_cost wp10 wp20) 10) (= (move_cost wp20 wp10) 10)
    (= (move_cost wp20 wp30) 10) (= (move_cost wp30 wp20) 10)
    (= (move_cost wp01 wp11) 10) (= (move_cost wp11 wp01) 10)
    (= (move_cost wp11 wp21) 10) (= (move_cost wp21 wp11) 10)
    (= (move_cost wp21 wp31) 10) (= (move_cost wp31 wp21) 10)
    (= (move_cost wp02 wp12) 10) (= (move_cost wp12 wp02) 10)
    (= (move_cost wp12 wp22) 10) (= (move_cost wp22 wp12) 10)
    (= (move_cost wp22 wp32) 10) (= (move_cost wp32 wp22) 10)
    (= (move_cost wp03 wp13) 10) (= (move_cost wp13 wp03) 10)
    (= (move_cost wp13 wp23) 10) (= (move_cost wp23 wp13) 10)
    (= (move_cost wp23 wp33) 10) (= (move_cost wp33 wp23) 10)

    ; Costes Horizontales (más baratos)
    (= (move_cost wp00 wp01) 5) (= (move_cost wp01 wp00) 5)
    (= (move_cost wp01 wp02) 5) (= (move_cost wp02 wp01) 5)
    (= (move_cost wp02 wp03) 5) (= (move_cost wp03 wp02) 5)
    (= (move_cost wp10 wp11) 5) (= (move_cost wp11 wp10) 5)
    (= (move_cost wp11 wp12) 5) (= (move_cost wp12 wp11) 5)
    (= (move_cost wp12 wp13) 5) (= (move_cost wp13 wp12) 5)
    (= (move_cost wp20 wp21) 5) (= (move_cost wp21 wp20) 5)
    (= (move_cost wp21 wp22) 5) (= (move_cost wp22 wp21) 5)
    (= (move_cost wp22 wp23) 5) (= (move_cost wp23 wp22) 5)
    (= (move_cost wp30 wp31) 5) (= (move_cost wp31 wp30) 5)
    (= (move_cost wp31 wp32) 5) (= (move_cost wp32 wp31) 5)
    (= (move_cost wp32 wp33) 5) (= (move_cost wp33 wp32) 5)
  )

  (:goal (and
      (box_at box1 wp21)
  ))

  (:metric maximize (battery-level kenny))
)
