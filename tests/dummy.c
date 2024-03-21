#include <gtk/gtk.h>

static void
print_hello (GtkWidget *widget, gpointer data) {
    g_print("%s\n", (char *)data);
}

static void
activate (GtkApplication *app, gpointer user_data) {
    GtkWidget *window;
    GtkWidget *button[12];
    char str[12][256];

    window = gtk_application_window_new (app);
    gtk_window_set_title (GTK_WINDOW (window), "Window");
    gtk_window_set_default_size (GTK_WINDOW (window), 200, 200);

    for(int n=0;n<12;n++) {
        snprintf(str[n], 256, "Hello %d", n);
        button[n] = gtk_button_new_with_label(str[n]);
        g_signal_connect (button[n], "clicked", G_CALLBACK (print_hello), str[n]);
        gtk_window_set_child (GTK_WINDOW (window), button[n]);
    }

  gtk_window_present (GTK_WINDOW (window));
}

int
main (int    argc, char **argv) {
  GtkApplication *app;
  int status;

  app = gtk_application_new ("org.gtk.example", 0);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  return status;
}
