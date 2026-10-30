from django.urls import path

from . import views

app_name = "marketing"

urlpatterns = [
    path("", views.home, name="home"),
    path("features/", views.page, {"page": "features"}, name="features"),
    path("how-it-works/", views.page, {"page": "how_it_works"}, name="how_it_works"),
    path("pricing/", views.page, {"page": "pricing"}, name="pricing"),
    path("apps/", views.page, {"page": "apps"}, name="apps"),
    path("install/", views.page, {"page": "installation"}, name="installation"),
    path("support/", views.support, name="support"),
    path("support/faq/", views.faq, name="faq"),
    path("support/articles/", views.articles, name="articles"),
    path("support/articles/<slug:slug>/", views.article, name="article"),
    path("open-source/", views.page, {"page": "open_source"}, name="open_source"),
    path("legal/", views.page, {"page": "legal"}, name="legal"),
    path("legal/terms/", views.page, {"page": "terms"}, name="terms"),
    path("legal/privacy/", views.page, {"page": "privacy"}, name="privacy"),
    path("legal/refunds/", views.page, {"page": "refunds"}, name="refunds"),
    path("legal/beta-terms/", views.page, {"page": "beta_terms"}, name="beta_terms"),
    path("legal/cookies/", views.page, {"page": "cookies"}, name="cookies"),
    path("checkout/", views.page, {"page": "checkout"}, name="checkout"),
    path("welcome/", views.page, {"page": "welcome"}, name="welcome"),
    path("account/", views.closed, {"destination": "account"}, name="account"),
    path("downloads/", views.closed, {"destination": "downloads"}, name="downloads"),
]
