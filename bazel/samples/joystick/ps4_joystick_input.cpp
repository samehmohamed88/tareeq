#include <SDL2/SDL.h>
#include <iostream>

int main() {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_GameController *controller = nullptr;
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            controller = SDL_GameControllerOpen(i);
            if (controller) {
                std::cout << "Controller found: " << SDL_GameControllerName(controller) << std::endl;
                break;
            } else {
                std::cerr << "Could not open gamecontroller " << i << ": " << SDL_GetError() << std::endl;
            }
        }
    }

    if (!controller) {
        std::cerr << "No game controllers found." << std::endl;
        SDL_Quit();
        return 1;
    }

    // Main loop
    bool running = true;
    SDL_Event e;
    while (running) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                running = false;
            } else if (e.type == SDL_CONTROLLERBUTTONDOWN) {
                std::cout << "Controller button pressed: " << static_cast<int>(e.cbutton.button) << std::endl;
            }
        }
    }

    SDL_GameControllerClose(controller);
    SDL_Quit();
    return 0;
}
