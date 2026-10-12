from django.urls import path

from . import views

app_name = "marketing"

urlpatterns = [
    path("", views.home, name="home"),
    path("features/", views.features, name="features"),
    path("pricing/", views.pricing, name="pricing"),
    path("account/", views.closed, {"destination": "account"}, name="account"),
    path("downloads/", views.closed, {"destination": "downloads"}, name="downloads"),
]
