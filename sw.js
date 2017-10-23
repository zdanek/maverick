/* ===========================================================
 * docsify sw.js
 * ===========================================================
 * Copyright 2016 @huxpro
 * Licensed under Apache 2.0
 * Register service worker.
 * ========================================================== */

const RUNTIME = 'docsify-v1'
const HOSTNAME_WHITELIST = [
  self.location.hostname,
  'fonts.gstatic.com',
  'fonts.googleapis.com',
  'unpkg.com'
]
const filesToInstall = [
  '.',
  '/web/maverick-docs/README.md',
  '/web/maverick-docs/_sidebar.md',
  '/web/maverick-docs/_navbar.md',
  '/web/maverick-docs/_coverpage.md'
]
const filesContent = [
  '/web/maverick-docs/about.md',
  //
  '/web/maverick-docs/devdocs/intro.md',
  '/web/maverick-docs/devdocs/distimages.md',
  //
  '/web/maverick-docs/modules/analysis.md',
  '/web/maverick-docs/modules/base.md',
  '/web/maverick-docs/modules/cloud9.md',
  '/web/maverick-docs/modules/desktop.md',
  '/web/maverick-docs/modules/dev.md',
  '/web/maverick-docs/modules/fc.md',
  '/web/maverick-docs/modules/hardware.md',
  '/web/maverick-docs/modules/intelligence.md',
  '/web/maverick-docs/modules/intro.md',
  '/web/maverick-docs/modules/mavlink.md',
  '/web/maverick-docs/modules/network.md',
  '/web/maverick-docs/modules/ros.md',
  '/web/maverick-docs/modules/security.md',
  '/web/maverick-docs/modules/vision.md',
  //
  '/web/maverick-docs/media/joule-thermal.jpg',
  '/web/maverick-docs/media/maverick-architecture.svg',
  '/web/maverick-docs/media/maverick-logo.svg',
  '/web/maverick-docs/media/maverick-snapshots.jpg',
  '/web/maverick-docs/media/mavlink-proxy.svg',
  '/web/maverick-docs/media/network-connection-sharing.svg',
  '/web/maverick-docs/media/precland1.png',
  '/web/maverick-docs/media/raspberryzw-seek.jpg',
  '/web/maverick-docs/media/seekthermal-pic.jpg',
  '/web/maverick-docs/media/thermal-lunch.jpg',
  '/web/maverick-docs/media/analysis/dashboard-switcher.jpg',
  '/web/maverick-docs/media/analysis/dashboard-zoomin.jpg',
  '/web/maverick-docs/media/analysis/error-index.jpg',
  '/web/maverick-docs/media/analysis/flightdata-dashboard.jpg',
  '/web/maverick-docs/media/analysis/logs-index.jpg',
  '/web/maverick-docs/media/analysis/panel.png'
]

// The Util Function to hack URLs of intercepted requests
const getFixedUrl = (req) => {
  var now = Date.now()
  var url = new URL(req.url)

  // 1. fixed http URL
  // Just keep syncing with location.protocol
  // fetch(httpURL) belongs to active mixed content.
  // And fetch(httpRequest) is not supported yet.
  url.protocol = self.location.protocol

  // 2. add query for caching-busting.
  // Github Pages served with Cache-Control: max-age=600
  // max-age on mutable content is error-prone, with SW life of bugs can even extend.
  // Until cache mode of Fetch API landed, we have to workaround cache-busting with query string.
  // Cache-Control-Bug: https://bugs.chromium.org/p/chromium/issues/detail?id=453190
  if (url.hostname === self.location.hostname) {
    url.search += (url.search ? '&' : '?') + 'cache-bust=' + now
  }
  return url.href
}

self.addEventListener('install', event => {
  console.log("Installing content into cache")
  event.waitUntil(
    caches.open(RUNTIME).then(function(cache) {
      cache.addAll(
        filesContent
      )
      return cache.addAll(
        filesToInstall
      )
    })
  )
})

/**
 *  @Lifecycle Activate
 *  New one activated when old isnt being used.
 *
 *  waitUntil(): activating ====> activated
 */
self.addEventListener('activate', event => {
  event.waitUntil(self.clients.claim())
})

/**
 *  @Functional Fetch
 *  All network requests are being intercepted here.
 *
 *  void respondWith(Promise<Response> r)
 */
self.addEventListener('fetch', event => {
  // Skip some of cross-origin requests, like those for Google Analytics.
  if (HOSTNAME_WHITELIST.indexOf(new URL(event.request.url).hostname) > -1) {
    // Stale-while-revalidate
    // similar to HTTP's stale-while-revalidate: https://www.mnot.net/blog/2007/12/12/stale
    // Upgrade from Jake's to Surma's: https://gist.github.com/surma/eb441223daaedf880801ad80006389f1
    const cached = caches.match(event.request)
    const fixedUrl = getFixedUrl(event.request)
    const fetched = fetch(fixedUrl, { cache: 'no-store' })
    const fetchedCopy = fetched.then(resp => resp.clone())

    // Call respondWith() with whatever we get first.
    // If the fetch fails (e.g disconnected), wait for the cache.
    // If there’s nothing in cache, wait for the fetch.
    // If neither yields a response, return offline pages.
    event.respondWith(
      Promise.race([fetched.catch(_ => cached), cached])
        .then(resp => resp || fetched)
        .catch(_ => { /* eat any errors */ })
    )

    // Update the cache with the version we fetched (only for ok status)
    event.waitUntil(
      Promise.all([fetchedCopy, caches.open(RUNTIME)])
        .then(([response, cache]) => response.ok && cache.put(event.request, response))
        .catch(_ => { /* eat any errors */ })
    )
  }
})