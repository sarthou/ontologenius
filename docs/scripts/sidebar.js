document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize External Sidebars (Fetches sidebar-cpp.html, etc.)
    initExternalSidebars();

    // 2. Initialize On-Page Table of Contents (Scans <h2> tags)
    initPageToc();

    // 3. Initialize Method Summary Table
    initSummaryTable();
});

// Helper: Automatically detects GitHub Pages subpath (e.g., "/my-repo" on "username.github.io/my-repo/")
const getBasePath = () => {
    const pathSegments = window.location.pathname.split('/').filter(Boolean);
    if (window.location.hostname.endsWith('github.io') && pathSegments.length > 0) {
        return '/' + pathSegments[0];
    }
    return '';
};

const basePath = getBasePath();

// Helper: Fixes root-relative paths ("/page.html" -> "/my-repo/page.html") on GitHub Pages
const fixPath = (path) => {
    if (path && path.startsWith('/') && !path.startsWith(basePath + '/')) {
        return basePath + path;
    }
    return path;
};

function initExternalSidebars() {
    const containers = document.querySelectorAll('[data-sidebar]');
    if (containers.length === 0) return;

    const normalize = (urlStr) => {
        try {
            const url = new URL(fixPath(urlStr), window.location.href);
            return url.pathname
                .toLowerCase()
                .replace(/\/$/, '')
                .replace(/\.html?$/i, '');
        } catch (e) {
            return '';
        }
    };

    const currentNormalized = normalize(window.location.href);

    containers.forEach(container => {
        const sidebarFile = container.getAttribute('data-sidebar');
        if (!sidebarFile) return;

        // Fix path for fetch if it starts with '/'
        fetch(fixPath(sidebarFile))
            .then(response => {
                if (!response.ok) throw new Error(`Could not load: ${sidebarFile}`);
                return response.text();
            })
            .then(html => {
                let processedHtml = html;

                // Automatically replace all {{datasetKey}} placeholders with data-* values
                Object.entries(container.dataset).forEach(([key, value]) => {
                    if (key === 'sidebar') return; // Skip the source filename attribute

                    // Creates regex matching {{title}}, {{title }}, etc.
                    const regex = new RegExp(`{{\\s*${key}\\s*}}`, 'g');
                    processedHtml = processedHtml.replace(regex, value);
                });

                container.innerHTML = processedHtml;

                // Initialize Pagefind if #search element exists inside injected HTML
                const searchElement = container.querySelector('#search');
                if (searchElement && typeof PagefindUI !== 'undefined') {
                    // Read data-search-base-url or fallback to fixPath('/') for GitHub Pages compatibility
                    const baseUrl = container.getAttribute('data-search-base-url') || fixPath('/') || '/';

                    new PagefindUI({
                        element: searchElement,
                        baseUrl: baseUrl,
                        showImages: false
                    });
                }

                // Highlight active link
                const links = container.querySelectorAll('a');
                links.forEach(link => {
                    const originalHref = link.getAttribute('href');
                    if (!originalHref) return;

                    // Fix root links on GitHub Pages (e.g. href="/ActionClient.html")
                    if (originalHref.startsWith('/')) {
                        link.setAttribute('href', fixPath(originalHref));
                    }

                    // Highlight active page
                    if (normalize(originalHref) === currentNormalized) {
                        link.style.fontWeight = 'bold';
                        link.classList.add('active');
                        if (link.parentElement && link.parentElement.tagName === 'LI') {
                            link.parentElement.classList.add('active-item');
                        }
                    }
                });

                const imgs = container.querySelectorAll('img');
                imgs.forEach(img => {
                    const originalSrc = img.getAttribute('src');
                    if (!originalSrc) return;

                    if (originalSrc.startsWith('/')) {
                        img.setAttribute('src', fixPath(originalSrc));
                    }
                });
            })
            .catch(error => console.error('Sidebar load error:', error));
    });
}

function initPageToc() {
    const tocContainers = document.querySelectorAll('[data-toc]');
    if (tocContainers.length === 0) return;

    // Find all H2 headings on the page
    const headings = document.querySelectorAll('main h2, article h2, body h2');
    if (headings.length === 0) return;

    let listItemsHtml = '';

    headings.forEach(h2 => {
        // 1. Auto-generate an ID if the <h2> doesn't have one
        if (!h2.id) {
            h2.id = h2.textContent
                .trim()
                .toLowerCase()
                .replace(/[^a-z0-9 -]/g, '') // remove special characters
                .replace(/\s+/g, '-')        // replace spaces with hyphens
                .replace(/-+/g, '-');        // collapse multiple hyphens
        }

        // 2. Create the <li> item matching your template
        listItemsHtml += `<li class="level1"><a href="#${h2.id}">${h2.textContent.trim()}</a></li>\n`;
    });

    // Inject the final HTML structure matching your layout
    tocContainers.forEach(container => {
        const customTitle = container.getAttribute('data-toc-title') || 'Contents';

        container.innerHTML = `
      <div class="contents class-content">
        <h3>${customTitle}</h3>
        <ul style="display: block;">
          ${listItemsHtml}
        </ul>
      </div>
    `;
    });
}

function initSummaryTable() {
    const tableContainers = document.querySelectorAll('[data-summary-table]');
    if (tableContainers.length === 0) return;

    const fnHeaders = document.querySelectorAll('h3.fn');
    if (fnHeaders.length === 0) return;

    const pageFile = window.location.pathname.split('/').pop() || '';
    let tableRowsHtml = '';

    fnHeaders.forEach(h3 => {
        // 1. Get anchor ID
        const anchorId = h3.id || h3.querySelector('a[name]')?.getAttribute('name') || '';
        const href = pageFile ? `${pageFile}#${anchorId}` : `#${anchorId}`;

        // 2. Find method name span
        const nameSpan = h3.querySelector('span.name');
        if (!nameSpan) return;

        const methodName = nameSpan.textContent.trim();

        // 3. Separate return type nodes from parameter nodes
        let returnTypeHtml = '';
        let paramsHtml = '';
        let passedNameSpan = false;

        h3.childNodes.forEach(node => {
            // Ignore inner <a name="..."> anchor tags
            if (node.nodeType === Node.ELEMENT_NODE && node.tagName === 'A' && node.getAttribute('name')) {
                return;
            }

            if (node === nameSpan) {
                passedNameSpan = true;
                return;
            }

            const nodeHtml = node.nodeType === Node.ELEMENT_NODE ? node.outerHTML : node.textContent;

            if (!passedNameSpan) {
                returnTypeHtml += nodeHtml;
            } else {
                paramsHtml += nodeHtml;
            }
        });

        // Fix 1: Strip ONLY the class prefix (e.g. FeederPublisher::) at the end of the return type string.
        // This preserves std::, onto_ros::, etc.
        returnTypeHtml = returnTypeHtml.replace(/[A-Za-z0-9_]+::\s*$/, '').trim();

        // Fix 2: Remove class="type" and class="hidden_type" attributes from return type tags
        returnTypeHtml = returnTypeHtml.replace(/\s*class=["'](?:type|hidden_type)["']/gi, '');

        // 4. Build table row
        tableRowsHtml += `
      <tr>
        <td class="memItemLeft rightAlign topAlign">${returnTypeHtml || ' '}</td>
        <td class="memItemRight bottomAlign"><b><a href="${href}">${methodName}</a></b>${paramsHtml.trim()}</td>
      </tr>`;
    });

    // Inject table
    tableContainers.forEach(container => {
        container.innerHTML = `
      <div class="table">
        <table class="alignedsummary">
          <tbody>
            ${tableRowsHtml}
          </tbody>
        </table>
      </div>
    `;
    });
}