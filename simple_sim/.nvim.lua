-- run `:luafile .nvim.lua` after startup to load this file
-- Or auto load by using 'exrc' (vim.o.exrc = true)
-- This uses https://github.com/mrcjkb/rustaceanvim or vim.lsp

local config = {
	["rust-analyzer"] = {
		cargo = {
			allFeatures = false,
			features = { "default" },
		},
	},
}

-- For rustaceanvim
local rustaceanvim_default_config = vim.g.rustaceanvim or {}
vim.g.rustaceanvim =
	vim.tbl_deep_extend("force", rustaceanvim_default_config, { server = { default_settings = config } })

-- For vim.lsp
-- local lsp_default_config = vim.lsp.config["rust-analyzer"] or {}
-- vim.lsp.config["rust-analyzer"] = vim.tbl_deep_extend("force", lsp_default_config, { settings = config })
