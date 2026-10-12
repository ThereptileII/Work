from django.http import Http404
from django.shortcuts import render
from django.views.decorators.http import require_GET

from .content import ARTICLES


def search_text(article):
    parts = [article["title"], article["category"], article["summary"]]
    for section in article["sections"]:
        parts.extend((section["title"], section.get("body", "")))
        parts.extend(section.get("bullets", ()))
    return " ".join(parts)


@require_GET
def home(request):
    return render(request, "marketing/home.html", {"nav_active": "home"})


@require_GET
def features(request):
    return render(request, "marketing/features.html", {"nav_active": "features"})


@require_GET
def pricing(request):
    return render(request, "marketing/pricing.html", {"nav_active": "pricing"})


@require_GET
def closed(request, destination):
    return render(request, "marketing/closed.html", {"destination": destination})


@require_GET
def support(request):
    query = request.GET.get("q", "")[:120].strip()
    terms = query.casefold().split()
    articles = tuple(dict(article, search_text=search_text(article)) for article in ARTICLES
                     if all(term in search_text(article).casefold() for term in terms))
    return render(request, "marketing/support.html", {"articles": articles, "query": query,
                                                     "nav_active": "support"})


@require_GET
def article(request, slug):
    selected = next((item for item in ARTICLES if item["slug"] == slug), None)
    if selected is None:
        raise Http404("Guide not found")
    return render(request, "marketing/article.html", {"article": selected, "nav_active": "support"})


@require_GET
def information(request, page):
    # Called with fixed route kwargs, never an external template path.
    if page not in ("installation", "open_source", "beta_terms", "privacy"):
        raise Http404("Page not found")
    return render(request, f"marketing/{page}.html", {"nav_active": page})
