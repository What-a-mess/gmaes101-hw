//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include <mutex>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int process = 0;
    std::mutex process_mutex;

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";
    int temp_process;

    auto render_block = [&](int sx, int sy, int ex, int ey) {
        for (uint32_t j = sy; j < ey; ++j) {
            int m = j * scene.width + sx;
            for (uint32_t i = sx; i < ex; ++i) {
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++){
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                }
                m++;
                process_mutex.lock();
                process++;
                UpdateProgress(1.0 * process / scene.width / scene.height);
                process_mutex.unlock();
            }
        }
    };

    const int X_BLOCK = 5, Y_BLOCK = 5;

    std::thread render_threads[X_BLOCK * Y_BLOCK];

    int x_step = scene.width / X_BLOCK + 1, y_step = scene.height / Y_BLOCK + 1;

    for (int i = 0; i < X_BLOCK; i++) {
        int sx = i * x_step, ex = std::min(sx + x_step, scene.width);
        for (int j = 0; j < Y_BLOCK; j++) {
            int sy = j * y_step, ey = std::min(sy + y_step, scene.height);
            render_threads[i * X_BLOCK + j] = std::thread(render_block, sx, sy, ex, ey);
        }
    }

    for (int i = 0; i < X_BLOCK * Y_BLOCK; i++) {
        render_threads[i].join();
    }

    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
