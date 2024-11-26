import { createRouter, createWebHistory } from "vue-router";
import ControllerView from "@/views/ControllerView.vue";
import VisualizationView from "@/views/VisualizationView.vue";
import StatusView from "@/views/StatusView.vue";
import LogView from "@/views/LogView.vue";

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: "/",
      redirect: "/controller",
    },
    {
      path: "/controller",
      name: "controller",
      component: ControllerView,
    },
    {
      path: "/visualization",
      name: "visualization",
      component: VisualizationView,
    },
    {
      path: "/status",
      name: "status",
      component: StatusView,
    },
    {
      path: "/log",
      name: "log",
      component: LogView,
    },
  ],
});

export default router;
