import { Component } from 'react'
import type { ErrorInfo, ReactNode } from 'react'

/**
 * Keeps one broken page from taking the whole dashboard with it. React unmounts the entire tree
 * when a render throws, so a single bad component left an operator staring at a blank window with
 * no header, no navigation and no way back. This replaces the page's content and nothing else.
 */
export class PageErrorBoundary extends Component<{ children: ReactNode }, { error: Error | null }> {
  state: { error: Error | null } = { error: null }

  static getDerivedStateFromError(error: Error) {
    return { error }
  }

  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error('page failed to render', error, info.componentStack)
  }

  render() {
    if (!this.state.error) return this.props.children
    return (
      <div className="flex h-full items-center justify-center p-6">
        <div className="max-w-md rounded-xl border border-[#e2707a]/40 bg-[#2a2030]/80 p-5 text-[13px] leading-6 text-white/80">
          <div className="mb-2 text-[13px] font-bold uppercase tracking-[0.09em] text-[#ffb9bf]">
            This page stopped
          </div>
          <p className="m-0">
            It failed while rendering, so the dashboard is showing this instead of a blank window.
            Every other page still works — pick one from the sidebar.
          </p>
          <p className="mb-0 mt-3 font-mono text-[12px] text-white/60">{this.state.error.message}</p>
          <button type="button" onClick={() => this.setState({ error: null })}
                  className="mt-4 rounded border border-white/20 bg-white/10 px-3 py-2 text-[12px] uppercase tracking-[0.09em] transition hover:bg-white/20">
            Try again
          </button>
        </div>
      </div>
    )
  }
}
