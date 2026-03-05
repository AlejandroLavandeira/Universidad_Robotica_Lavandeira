(define (problem problem2)
  ; --- CAMBIO CRÍTICO ---
  (:domain domain2)

  (:objects
    kenny - robot
    wp00 wp01 wp02 wp03
    wp10 wp11 wp12 wp13
    wp20 wp21 wp22 wp23
    wp30 wp31 wp32 wp33 - waypoint
    up down left right - orientation
  )

  (:init
    ; El estado inicial es idéntico al del problema anterior
    (at kenny wp00)
    (robot_has_orientation kenny down)

    ; (Aquí va la definición completa del mapa, que no cambia)
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
  )

  (:goal (and
      ; --- NUEVO OBJETIVO ---
      (at kenny wp31)
  ))
)
