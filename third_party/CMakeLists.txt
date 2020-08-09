
add_library(imgui SHARED
  imgui/imconfig.h
  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_internal.h
  imgui/imgui_widgets.cpp
  imgui/imgui.cpp
  imgui/imgui.h
  imgui/imstb_rectpack.h
  imgui/imstb_textedit.h
  imgui/imstb_truetype.h
  imgui/examples/imgui_impl_glfw.h
  imgui/examples/imgui_impl_glfw.cpp
  imgui/examples/imgui_impl_opengl3.cpp
  imgui/examples/imgui_impl_opengl3.h
  )

target_link_libraries(imgui
  ${GLEW_LIBRARIES}
  ${OPENGL_gl_LIBRARY}
  glfw
  )
