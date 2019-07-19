#ifndef SDL_IMAGE_VIEWING_HPP
#define SDL_IMAGE_VIEWING_HPP

#include <SDL/SDL.h>
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

namespace sdl_image_viewing {
  static int filter(const SDL_Event * event)
  { return event->type == SDL_QUIT; }

  static bool init_app(const char * name, SDL_Surface * icon = 0, uint32_t flags = SDL_INIT_VIDEO )
  {
    atexit(SDL_Quit);
    if(SDL_Init(flags) < 0)
      return 0;

    SDL_WM_SetCaption(name, name);
    SDL_WM_SetIcon(icon, NULL);

    SDL_SetEventFilter( filter );
    return 1;
  }

  // caller is responsible of deallocating buffer and surface (surface must be free'd first)
  static SDL_Surface* create_buffer_and_surface( int w, int h, unsigned char*& rgb_buffer )
  {
    rgb_buffer = new unsigned char[ w*h*3 ];

    SDL_Surface * data_sf = SDL_CreateRGBSurfaceFrom(
                            rgb_buffer, w, h, 24, w * 3,
                            0x00ff0000, 0x0000ff00, 0x000000ff, 0 );
    return data_sf;
  }

  static void render(SDL_Surface * sf)
  {
    SDL_Surface * screen = SDL_GetVideoSurface();
    if(SDL_BlitSurface(sf, NULL, screen, NULL) == 0)
      SDL_UpdateRect(screen, 0, 0, 0, 0);
  }


}
#endif // SDL_IMAGE_VIEWING_HPP
