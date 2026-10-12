from django.shortcuts import render


def home(request):
    return render(request, "marketing/home.html", {"nav_active": "home"})


def features(request):
    return render(request, "marketing/features.html", {"nav_active": "features"})


def pricing(request):
    return render(request, "marketing/pricing.html", {"nav_active": "pricing"})


def closed(request, destination):
    return render(request, "marketing/closed.html", {"destination": destination})
