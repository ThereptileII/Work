from django.urls import include, path


urlpatterns = [path("auth/", include("skager_web.accounts.urls"))]
