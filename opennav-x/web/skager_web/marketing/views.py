import json

from django.http import Http404
from django.shortcuts import render
from django.urls import reverse
from django.utils.safestring import mark_safe
from django.views.decorators.http import require_GET

from .content import ARTICLES, CATEGORIES, FAQS

# Pages rendered from a fixed template name, never from request input.
PAGES = {
    "features": "features", "how_it_works": "how_it_works", "pricing": "pricing",
    "installation": "installation", "open_source": "open_source", "legal": "legal",
    "terms": "legal_terms", "privacy": "legal_privacy", "refunds": "legal_refunds",
    "beta_terms": "legal_beta_terms", "cookies": "legal_cookies",
    "checkout": "checkout", "welcome": "welcome",
}


def search_text(article):
    parts = [article["title"], article["category"], article["summary"]]
    for section in article["sections"]:
        body = section.get("body", "")
        parts.append(section["title"])
        parts.extend(body if isinstance(body, tuple) else (body,))
        parts.extend(section.get("bullets", ()))
    return " ".join(parts)


def reading_minutes(article):
    return max(1, round(len(search_text(article).split()) / 200))


def with_meta(article):
    sections = tuple(dict(section, paragraphs=section["body"] if isinstance(section.get("body"), tuple)
                          else ((section["body"],) if section.get("body") else ()))
                     for section in article["sections"])
    return dict(article, sections=sections, minutes=reading_minutes(article))


def json_ld(data):
    # Owned content only; "<" is escaped so the block can never close its script element.
    return mark_safe(json.dumps(data, ensure_ascii=False).replace("<", "\\u003c"))


def faq_link(item):
    link = item.get("link")
    if not link:
        return None
    name, label, *args = link
    return {"url": reverse(f"marketing:{name}", args=args), "label": label}


def faq_text(item):
    return f"{item['question']} {item['answer']}"


def matches(text, terms):
    folded = text.casefold()
    return all(term in folded for term in terms)


@require_GET
def home(request):
    return render(request, "marketing/home.html", {"nav_active": "home"})


@require_GET
def page(request, page):
    if page not in PAGES:
        raise Http404("Page not found")
    return render(request, f"marketing/{PAGES[page]}.html", {"nav_active": page})


@require_GET
def closed(request, destination):
    return render(request, "marketing/closed.html", {"destination": destination})


@require_GET
def support(request):
    query = request.GET.get("q", "")[:120].strip()
    terms = query.casefold().split()
    articles = tuple(dict(article, search_text=search_text(article)) for article in ARTICLES
                     if matches(search_text(article), terms))
    faqs = tuple(item for group in FAQS for item in group["items"] if terms and matches(faq_text(item), terms))
    return render(request, "marketing/support.html", {
        "articles": tuple(with_meta(a) for a in articles), "query": query, "nav_active": "support",
        "featured": tuple(with_meta(a) for a in ARTICLES if a.get("featured"))[:6],
        "faqs": tuple(dict(item, href=faq_link(item)) for item in faqs),
        "result_count": len(articles) + len(faqs),
    })


@require_GET
def faq(request):
    groups = tuple({"name": group["name"], "items": tuple(dict(item, href=faq_link(item)) for item in group["items"])}
                   for group in FAQS)
    schema = {"@context": "https://schema.org", "@type": "FAQPage", "mainEntity": [
        {"@type": "Question", "name": item["question"],
         "acceptedAnswer": {"@type": "Answer", "text": item["answer"]}}
        for group in FAQS for item in group["items"]]}
    return render(request, "marketing/faq.html", {"groups": groups, "schema": json_ld(schema), "nav_active": "support"})


@require_GET
def articles(request):
    grouped = tuple({"name": name, "description": description,
                     "articles": tuple(with_meta(a) for a in ARTICLES if a["category"] == name)}
                    for name, description in CATEGORIES)
    return render(request, "marketing/articles.html", {"categories": grouped, "nav_active": "support"})


@require_GET
def article(request, slug):
    selected = next((item for item in ARTICLES if item["slug"] == slug), None)
    if selected is None:
        raise Http404("Article not found")
    related = tuple(with_meta(a) for a in ARTICLES
                    if a["category"] == selected["category"] and a["slug"] != slug)[:3]
    schema = {"@context": "https://schema.org", "@type": "Article", "headline": selected["title"],
              "description": selected["summary"], "dateModified": selected["updated"],
              "author": {"@type": "Organization", "name": "SKAGER"},
              "publisher": {"@type": "Organization", "name": "SKAGER"}}
    return render(request, "marketing/article.html", {"article": with_meta(selected), "related": related,
                                                     "schema": json_ld(schema), "nav_active": "support"})
