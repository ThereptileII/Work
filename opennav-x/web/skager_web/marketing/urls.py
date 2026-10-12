from django.urls import path

from . import views

app_name = "marketing"

urlpatterns = [
    path("", views.home, name="home"),
    path("features/", views.features, name="features"),
    path("pricing/", views.pricing, name="pricing"),
    path("support/", views.support, name="support"),
    path("support/<slug:slug>/", views.article, name="article"),
    path("install/", views.information, {"page": "installation"}, name="installation"),
    path("open-source/", views.information, {"page": "open_source"}, name="open_source"),
    path("beta-terms/", views.information, {"page": "beta_terms"}, name="beta_terms"),
    path("privacy/", views.information, {"page": "privacy"}, name="privacy"),
    path("account/", views.closed, {"destination": "account"}, name="account"),
    path("downloads/", views.closed, {"destination": "downloads"}, name="downloads"),
]
