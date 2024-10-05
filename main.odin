package main

import "core:fmt"
import "core:math"
import "core:math/rand"
import SDL "vendor:sdl2"


WINDOW_WIDTH  :: 1920
WINDOW_HEIGHT :: 1080
GRID_SIZE     :: 128
SCALE         :: WINDOW_WIDTH / GRID_SIZE

Fluid :: struct {
    size:      int,
    dt:        f32,
    diff:      f32,
    visc:      f32,
    s:         []f32,
    density:   []f32,
    vx:        []f32,
    vy:        []f32,
    vx0:       []f32,
    vy0:       []f32,
}

create_fluid :: proc(size: int, dt, diff, visc: f32) -> Fluid {
  assert(size >= 3, "Fluid size must be at least 3")  
  return Fluid{
        size  = size,
        dt    = dt,
        diff  = diff,
        visc  = visc,
        s     = make([]f32, size * size),
        density = make([]f32, size * size),
        vx    = make([]f32, size * size),
        vy    = make([]f32, size * size),
        vx0   = make([]f32, size * size),
        vy0   = make([]f32, size * size),
    }
}

destroy_fluid :: proc(fluid: ^Fluid) {
    delete(fluid.s)
    delete(fluid.density)
    delete(fluid.vx)
    delete(fluid.vy)
    delete(fluid.vx0)
    delete(fluid.vy0)
}

is_coord_valid :: proc(fluid: ^Fluid, x,y: int) -> bool {
  if x < 0 || x >= fluid.size || y < 0 || y >= fluid.size {
    return false
  }
  return true
}

add_density :: proc(fluid: ^Fluid, x, y: int, amount: f32) {
  if is_coord_valid(fluid, x, y) == false {
    return
  }
  index := x + y * fluid.size
  fluid.density[index] += amount
}

add_velocity :: proc(fluid: ^Fluid, x, y: int, amount_x, amount_y: f32) {
  if is_coord_valid(fluid, x, y) == false {
    return
  }
  index := x + y * fluid.size
  fluid.vx[index] += amount_x
  fluid.vy[index] += amount_y
}

diffuse :: proc(fluid: ^Fluid, b: int, x: []f32, x0: []f32, diff: f32) {
    a := fluid.dt * diff * f32(fluid.size - 2) * f32(fluid.size - 2)
    for _ in 0..<20 {
        for i in 1..<fluid.size-1 {
            for j in 1..<fluid.size-1 {
                x[i + j * fluid.size] =
                    (x0[i + j * fluid.size] +
                     a * (x[(i+1) + j * fluid.size] +
                          x[(i-1) + j * fluid.size] +
                          x[i + (j+1) * fluid.size] +
                          x[i + (j-1) * fluid.size])) / (1 + 4 * a)
            }
        }
    }
}

project :: proc(fluid: ^Fluid, vx, vy, p, div: []f32) {
    for j in 1..<fluid.size-1 {
        for i in 1..<fluid.size-1 {
            div[i + j * fluid.size] = -0.5 * (
                vx[(i+1) + j * fluid.size] -
                vx[(i-1) + j * fluid.size] +
                vy[i + (j+1) * fluid.size] -
                vy[i + (j-1) * fluid.size]
            ) / f32(fluid.size)
            p[i + j * fluid.size] = 0
        }
    }

    for _ in 0..<20 {
        for j in 1..<fluid.size-1 {
            for i in 1..<fluid.size-1 {
                p[i + j * fluid.size] = (div[i + j * fluid.size] +
                    p[(i+1) + j * fluid.size] +
                    p[(i-1) + j * fluid.size] +
                    p[i + (j+1) * fluid.size] +
                    p[i + (j-1) * fluid.size]) / 4
            }
        }
    }

    for j in 1..<fluid.size-1 {
        for i in 1..<fluid.size-1 {
            vx[i + j * fluid.size] -= 0.5 * (p[(i+1) + j * fluid.size] - p[(i-1) + j * fluid.size]) * f32(fluid.size)
            vy[i + j * fluid.size] -= 0.5 * (p[i + (j+1) * fluid.size] - p[i + (j-1) * fluid.size]) * f32(fluid.size)
        }
    }
}

advect :: proc(fluid: ^Fluid, b: int, d, d0, vx, vy: []f32) {
    dt0 := fluid.dt * f32(fluid.size - 2)
    for i in 1..<fluid.size-1 {
        for j in 1..<fluid.size-1 {
            x := f32(i) - dt0 * vx[i + j * fluid.size]
            y := f32(j) - dt0 * vy[i + j * fluid.size]
            
            if x < 0.5 do x = 0.5
            if x > f32(fluid.size) + 0.5 do x = f32(fluid.size) + 0.5
            i0 := int(x)
            i1 := i0 + 1
            
            if y < 0.5 do y = 0.5
            if y > f32(fluid.size) + 0.5 do y = f32(fluid.size) + 0.5
            j0 := int(y)
            j1 := j0 + 1
            
            s1 := x - f32(i0)
            s0 := 1 - s1
            t1 := y - f32(j0)
            t0 := 1 - t1
            
            d[i + j * fluid.size] =
                s0 * (t0 * d0[i0 + j0 * fluid.size] + t1 * d0[i0 + j1 * fluid.size]) +
                s1 * (t0 * d0[i1 + j0 * fluid.size] + t1 * d0[i1 + j1 * fluid.size])
        }
    }
}

fluid_step :: proc(fluid: ^Fluid) {
    diffuse(fluid, 1, fluid.vx0, fluid.vx, fluid.visc)
    diffuse(fluid, 2, fluid.vy0, fluid.vy, fluid.visc)
    
    project(fluid, fluid.vx0, fluid.vy0, fluid.vx, fluid.vy)
    
    advect(fluid, 1, fluid.vx, fluid.vx0, fluid.vx0, fluid.vy0)
    advect(fluid, 2, fluid.vy, fluid.vy0, fluid.vx0, fluid.vy0)
    
    project(fluid, fluid.vx, fluid.vy, fluid.vx0, fluid.vy0)
    
    diffuse(fluid, 0, fluid.s, fluid.density, fluid.diff)
    advect(fluid, 0, fluid.density, fluid.s, fluid.vx, fluid.vy)
}

main :: proc() {
  if err := SDL.Init({.VIDEO}); err != 0 {
    fmt.eprintln("Error initializing SDL: ", SDL.GetError())
    return
  }
  defer SDL.Quit()

  window := SDL.CreateWindow("Text Animation",
    SDL.WINDOWPOS_UNDEFINED, SDL.WINDOWPOS_UNDEFINED,
    WINDOW_WIDTH, WINDOW_HEIGHT,
    {.SHOWN})

  if window == nil {
    fmt.eprintln("Error creating window:", SDL.GetError())
  }
  defer SDL.DestroyWindow(window)

  renderer := SDL.CreateRenderer(window, -1, {.ACCELERATED, .PRESENTVSYNC})
  if renderer == nil {
    fmt.eprintln("Error creating renderer:", SDL.GetError())
    return
  }
  defer SDL.DestroyRenderer(renderer)

  fluid := create_fluid(GRID_SIZE, 0.1, 0, 0)
  assert(fluid.size == GRID_SIZE, "GRID_SIZE must match fluid size")
  defer destroy_fluid(&fluid)

  running := true
  for running {
    e: SDL.Event
    for SDL.PollEvent(&e) {
      #partial switch e.type {
      case .QUIT:
        running = false
      case .MOUSEMOTION:
        mouse_x, mouse_y: i32
        SDL.GetMouseState(&mouse_x, &mouse_y)
        add_density(&fluid, int(mouse_x / SCALE), int(mouse_y / SCALE), 100)
        add_velocity(&fluid, int(mouse_x / SCALE), int(mouse_y / SCALE), 5, 5)
      }
    }

    fluid_step(&fluid)

    SDL.SetRenderDrawColor(renderer, 0, 0, 0, 255)
    SDL.RenderClear(renderer)

    for i in 0..<GRID_SIZE {
      for j in 0..<GRID_SIZE {
        x := i * SCALE
        y := j * SCALE
        d := fluid.density[i + j * GRID_SIZE]
        c := u8(math.min(d * 255, 255))
        SDL.SetRenderDrawColor(renderer, c, c, c, 255)
        rect := SDL.Rect{i32(x), i32(y), i32(SCALE), i32(SCALE)}
        SDL.RenderFillRect(renderer, &rect)
      }
    }

    SDL.RenderPresent(renderer)
  }
}
