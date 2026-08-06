document.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize External Sidebars (Fetches sidebar-cpp.html, etc.)
    initExternalSidebars();

    // 2. Initialize On-Page Table of Contents (Scans <h2> tags)
    initPageToc();
});

function initExternalSidebars() {
    const containers = document.querySelectorAll('[data-sidebar]');
    if (containers.length === 0) return;

    const normalize = (urlStr) => {
        try {
            const url = new URL(urlStr, window.location.href);
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

        fetch(sidebarFile)
            .then(response => {
                if (!response.ok) throw new Error(`Could not load: ${sidebarFile}`);
                return response.text();
            })
            .then(html => {
                container.innerHTML = html;

                const links = container.querySelectorAll('a');
                links.forEach(link => {
                    const href = link.getAttribute('href');
                    if (href && normalize(href) === currentNormalized) {
                        link.style.fontWeight = 'bold';
                        link.classList.add('active');
                        if (link.parentElement.tagName === 'LI') {
                            link.parentElement.classList.add('active-item');
                        }
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

    // Build the list items
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