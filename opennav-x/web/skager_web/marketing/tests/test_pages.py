"""Focused informational-site boundaries. No database, vendors or live services."""
import re
from html.parser import HTMLParser
from urllib.parse import urlsplit

from django.test import SimpleTestCase
from django.urls import resolve, reverse

from skager_web.marketing.content import ARTICLES


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        self.links.extend(value for key, value in attrs if key == "href" and value)


class MarketingPages(SimpleTestCase):
    pages = ("home", "features", "how_it_works", "pricing", "installation", "support", "faq", "articles",
             "open_source", "legal", "terms", "privacy", "refunds", "beta_terms", "cookies", "account", "downloads", "checkout", "welcome")

    def test_pages_and_article_links_resolve_without_database_or_vendors(self):
        paths = [reverse("marketing:" + name) for name in self.pages]
        paths += [reverse("marketing:article", args=[a["slug"]]) for a in ARTICLES]
        for path in paths:
            with self.subTest(path=path):
                response = self.client.get(path)
                self.assertEqual(response.status_code, 200)
                parser = Links()
                parser.feed(response.content.decode())
                for href in parser.links:
                    parsed = urlsplit(href)
                    self.assertIn(parsed.scheme, ("", "https"))
                    if href.startswith("/") and not href.startswith("/static/"):
                        resolve(parsed.path)

    def test_search_matches_article_body_and_has_explicit_empty_state(self):
        result = self.client.get(reverse("marketing:support"), {"q": "transducer"})
        self.assertIn("sensors-and-stale-data", [a["slug"] for a in result.context["articles"]])
        faq = self.client.get(reverse("marketing:support"), {"q": "refund"})
        self.assertTrue(faq.context["faqs"])
        empty = self.client.get(reverse("marketing:support"), {"q": "zz-no-match-zz"})
        self.assertContains(empty, "No matching articles")
        self.assertEqual(empty.context["articles"], ())

    def test_search_is_bounded_and_escaped(self):
        response = self.client.get(reverse("marketing:support"), {"q": '<script>alert("x")</script>'})
        self.assertNotContains(response, '<script>alert("x")</script>', html=False)
        self.assertContains(response, "&lt;script&gt;")
        long = self.client.get(reverse("marketing:support"), {"q": "a" * 2000})
        self.assertEqual(len(long.context["query"]), 120)

    def test_unknown_article_is_not_a_file_or_template_lookup(self):
        self.assertEqual(self.client.get("/support/articles/not-a-guide/").status_code, 404)
        self.assertEqual(self.client.get("/support/articles/..%2Fprivacy/").status_code, 404)

    def test_information_pages_reject_posts(self):
        for name in self.pages:
            with self.subTest(name=name):
                self.assertEqual(self.client.post(reverse("marketing:" + name)).status_code, 405)

    def test_preview_security_headers_do_not_enable_authentication(self):
        response = self.client.get("/")
        self.assertIn("script-src 'self'", response["Content-Security-Policy"])
        self.assertIn("frame-ancestors 'none'", response["Content-Security-Policy"])
        self.assertEqual(response["X-Content-Type-Options"], "nosniff")
        self.assertEqual(response["X-Robots-Tag"], "noindex, nofollow")
        self.assertNotIn("sessionid", response.cookies)
        self.assertEqual(self.client.get("/auth/login/").status_code, 404)

    def test_accounts_come_only_from_checkout_and_licenses_are_not_paywalled(self):
        for name in ("account", "downloads"):
            response = self.client.get(reverse("marketing:" + name))
            self.assertContains(response, 'href="/auth/login/"')
            self.assertContains(response, 'href="/checkout/"')
            self.assertNotContains(response, 'type="password"')
            self.assertNotContains(response, 'type="email"')
        # No public page offers sign-up: an account is created only by a purchase.
        for name in self.pages:
            with self.subTest(name=name):
                html = self.client.get(reverse("marketing:" + name)).content.decode().lower()
                found = re.search(r"sign[ -]?up|create (?:an |your )?account|\bregister\b|/auth/signup|type=\"password\"", html)
                self.assertIsNone(found and found.group(0))
        checkout = self.client.get(reverse("marketing:checkout"))
        self.assertContains(checkout, 'id="checkout-email"')
        self.assertContains(checkout, "account is created with this email when you pay")
        self.assertContains(checkout, 'class="helm-scene"')
        self.assertContains(checkout, 'data-fit="1280x800"')
        self.assertNotContains(checkout, "<script>")
        pricing = self.client.get(reverse("marketing:pricing"))
        self.assertContains(pricing, 'class="cta plan-cta" href="/checkout/"')
        source = self.client.get(reverse("marketing:open_source"))
        self.assertContains(source, "https://opencpn.org/")
        self.assertContains(source, "GPL")

